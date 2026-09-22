"""ArduinoQuad-Walk / -Robust as a vectorized plain-MuJoCo environment.

Each environment is its own ``MjModel`` + state row; ``mujoco.rollout`` steps
all of them on a C++ thread pool, and everything else (commands, observations,
rewards, resets) is numpy over the batch. The step follows mjlab's
``ManagerBasedRlEnv.step`` term by term, so the policy sees the same MDP:

  action -> target = home + 0.25 a - encoder_bias
  4 physics substeps, each: push the target into the delay line, read it back
      4..7 physics steps late, PD + back-EMF damping + effort limit in MuJoCo,
      update foot air/contact time from the contact sensors
  terminations -> rewards (x dt) -> reset finished envs -> resample commands
  -> random pushes -> observations (noise, then a 3-step history per term)

Where it deliberately differs from mjlab (both are one physics substep, 5 ms):

* Rewards and terminations see the base, the feet, the contacts and the
  actuator force one physics substep stale, as in mjlab. The critic, however,
  gets the fresh base state but the stale foot heights and contacts, except
  for envs that were just reset (they get a fresh ``mj_forward``); mjlab runs
  one ``forward`` for every env before observing.
* ``joint_acc_l2`` uses the integrated acceleration (dv/dt) of the last
  substep: ``rollout`` does not return ``qacc``. Its weight is -2.5e-7.
"""

import copy
import math
import os

import mujoco
import numpy as np
import torch
from mujoco import rollout as mj_rollout
from rsl_rl.env import VecEnv
from tensordict import TensorDict

import task as T

_POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def _ranges(d: dict) -> np.ndarray:
    return np.array([d.get(k, (0.0, 0.0)) for k in _POSE_KEYS], dtype=np.float64)


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    """(N, 4) wxyz -> (N, 3, 3) body-to-world rotation."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((q.shape[0], 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - w * z)
    R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y)
    R[:, 2, 1] = 2 * (y * z + w * x)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def quat_from_euler_xyz(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """Same convention as mjlab's ``quat_from_euler_xyz`` (R = Rz Ry Rx), wxyz."""
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    return np.stack([
        cy * cr * cp + sy * sr * sp,
        cy * sr * cp - sy * cr * sp,
        cy * cr * sp + sy * sr * cp,
        sy * cr * cp - cy * sr * sp,
    ], axis=1)


def _terrain_tile(rng: np.random.Generator, n: int, size: float) -> np.ndarray:
    """One (n, n) height map [m] of the kinds env_cfgs.py uses for rough ground:
    flat 30 %, random-uniform 40 % (5-20 mm), waves 30 % (5-25 mm), each at a
    random difficulty. The same three kinds as mjlab's generator, not a copy of it.
    """
    a = T.TERRAIN_AMP
    kind = rng.choice(3, p=(0.3, 0.4, 0.3))
    level = rng.uniform()
    if kind == 0:
        return np.zeros((n, n))
    if kind == 1:
        amp = (0.005 + level * 0.015) * a
        step = max(0.005 * a, 0.005)
        coarse = rng.uniform(-amp, amp, size=(n // 2 + 1, n // 2 + 1))
        coarse = np.round(coarse / step) * step
        return np.kron(coarse, np.ones((2, 2)))[:n, :n]
    amp = (0.005 + level * 0.020) * a
    u = np.linspace(0.0, 2.0 * math.pi * 4, n)
    return amp * 0.5 * (np.sin(u)[None, :] + np.sin(u)[:, None])


class QuadEnv(VecEnv):
    """rsl_rl ``VecEnv`` over ``num_envs`` plain-MuJoCo worlds.

    Parameters
    ----------
    num_envs : int
        Number of parallel environments.
    robust : bool
        ``True`` for ArduinoQuad-Robust (off-nominal resets), else ArduinoQuad-Walk.
    play : bool
        mjlab's ``play=True``: no observation noise, no pushes, no command
        curriculum (full range) and no time limit.
    domain_rand : bool
        Apply the startup domain randomization. ``False`` gives every env the
        nominal robot, for evaluation.
    threads : int or None
        Size of the ``mujoco.rollout`` pool (default: every logical CPU).
    seed : int
        Seed for everything this env samples.
    """

    def __init__(self, num_envs: int, robust: bool = False, play: bool = False,
                 domain_rand: bool = True, threads: int | None = None, seed: int = 0) -> None:
        self.num_envs = num_envs
        self.num_actions = len(T.JOINT_NAMES)
        self.device = torch.device("cpu")
        self.robust = robust
        self.play = play
        self.rng = np.random.default_rng(seed)
        self.max_episode_length = (
            np.iinfo(np.int64).max // 2 if play
            else int(math.ceil(T.EPISODE_LENGTH_S / T.STEP_DT)))
        self.cfg = {"task": "ArduinoQuad-Robust" if robust else "ArduinoQuad-Walk",
                    "num_envs": num_envs, "play": play, "domain_rand": domain_rand}

        self._build_model()
        N, nj = num_envs, self.num_actions
        self.models = [copy.deepcopy(self.model) for _ in range(N)]
        self.encoder_bias = np.zeros((N, nj))
        if domain_rand:
            self._startup_domain_randomization()
        if T.TERRAIN == "rough":
            self._startup_terrain()

        self.threads = threads or mujoco_threads()
        self.pool = mj_rollout.Rollout(nthread=self.threads)
        self.datas = [mujoco.MjData(self.model) for _ in range(self.threads)]
        self._fwd = mujoco.MjData(self.model)
        m = self.model
        self.nstate = mujoco.mj_stateSize(m, mujoco.mjtState.mjSTATE_FULLPHYSICS.value)
        self.control_spec = mujoco.mjtState.mjSTATE_CTRL.value
        if T.ERFI:
            self.control_spec |= mujoco.mjtState.mjSTATE_QFRC_APPLIED.value
        ncontrol = mujoco.mj_stateSize(m, self.control_spec)
        self.state = np.zeros((N, self.nstate))
        self._ctrl = np.zeros((N, T.DECIMATION, ncontrol))
        self._x_out = np.zeros((N, T.DECIMATION, self.nstate))
        self._sd_out = np.zeros((N, T.DECIMATION, m.nsensordata))
        self.sd = np.zeros((N, m.nsensordata))

        self.default_joint_pos = np.array(T.DEFAULT_JOINT_POS)
        lo, hi = m.jnt_range[self.jnt_ids, 0], m.jnt_range[self.jnt_ids, 1]
        mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo) * T.SOFT_JOINT_POS_LIMIT_FACTOR
        self.soft_lo, self.soft_hi = mid - half, mid + half
        hip = np.array(["hip" in j for j in T.JOINT_NAMES])
        self.pose_std = {
            name: np.where(hip, d["hip"], d["knee"])
            for name, d in (("standing", T.STD_STANDING), ("walking", T.STD_WALKING),
                            ("running", T.STD_RUNNING))}

        self.action = np.zeros((N, nj))
        self.prev_action = np.zeros((N, nj))
        self._delay_hist = np.zeros((N, T.DELAY_MAX_LAG + 1, nj))
        self._delay_len = np.zeros(N, dtype=np.int64)
        self._delay_ptr = 0
        self.erfi_offset = np.zeros((N, nj))
        self.command = np.zeros((N, 3))
        self.cmd_time_left = np.zeros(N)
        self.is_standing = np.zeros(N, dtype=bool)
        self.cmd_lin_x, self.cmd_ang_z = T.CMD_CURRICULUM[0][1], T.CMD_CURRICULUM[0][2]
        if play:
            self.cmd_lin_x, self.cmd_ang_z = T.CMD_LIN_X, T.CMD_ANG_Z
        self.fixed_command: np.ndarray | None = None
        # Drawn once and then only after each push: in mjlab the timer of a
        # plain-function interval event is not restarted by an episode reset.
        self.push_time_left = self.rng.uniform(*T.PUSH_INTERVAL_S, size=N)
        self.episode_length = np.zeros(N, dtype=np.int64)
        self.common_step_counter = 0
        nf = len(T.FOOT_ORDER)
        # Foot air / contact timers are float32 and advance by the difference of
        # a float32 sim clock, exactly as in mjlab (warp): whether a contact that
        # began on the first substep of a control step still counts as a "first
        # contact" (< step_dt + 1e-8) depends on that rounding.
        f32 = np.float32
        self.sim_time = np.zeros(N, dtype=f32)
        self.last_time = np.zeros(N, dtype=f32)
        self.cur_air = np.zeros((N, nf), dtype=f32)
        self.last_air = np.zeros((N, nf), dtype=f32)
        self.cur_contact = np.zeros((N, nf), dtype=f32)
        self.last_contact = np.zeros((N, nf), dtype=f32)
        self.peak_heights = np.zeros((N, nf))
        self.vel_filtered = np.zeros((N, 2))
        self.joint_acc = np.zeros((N, nj))
        self.episode_sums = {k: np.zeros(N) for k in T.REWARD_WEIGHTS}

        self._build_obs_layout()
        self._need_backfill = np.ones(N, dtype=bool)
        self._actor_hist = np.zeros((N, T.ACTOR_HISTORY, self._actor_step_dim))
        self.extras: dict = {"log": {}}
        self._reset(np.arange(N))
        self._write_targets_after_reset(np.arange(N), np.zeros((N, nj)))
        self._observe(np.arange(N))

    # ------------------------------------------------------------------ model

    def _build_model(self) -> None:
        spec = mujoco.MjSpec.from_file(str(T.MJCF))
        for act in list(spec.actuators):
            spec.delete(act)
        for key in list(spec.keys):
            spec.delete(key)
        toes = set(T.TOE_GEOMS)
        # Only the four toe spheres collide; the meshes are visual and every env
        # copies the model, so they are dropped (the MJCF has explicit inertials).
        for g in list(spec.geoms):
            if g.name == "ground" or g.name in toes:
                continue
            spec.delete(g)
        for mesh in list(spec.meshes):
            spec.delete(mesh)
        for name in T.TOE_GEOMS:
            g = spec.geom(name)
            g.contype, g.conaffinity, g.condim, g.priority = 0, 1, T.TOE_CONDIM, 1
            g.friction[0] = T.TOE_FRICTION
            g.solimp[:3] = T.TOE_SOLIMP
        if T.TERRAIN == "rough":
            spec.delete(spec.geom("ground"))
            n, size = 161, 4.0   # 8 m x 8 m tile, 5 cm grid, per env
            spec.add_hfield(name="terrain", size=[size, size, 0.01, 0.1], nrow=n, ncol=n,
                            userdata=[0.0] * (n * n))
            spec.worldbody.add_geom(name="ground", type=mujoco.mjtGeom.mjGEOM_HFIELD,
                                    hfieldname="terrain", contype=1, conaffinity=1)
        for j in T.JOINT_NAMES:
            jt = spec.joint(j)
            jt.armature = T.ARMATURE
            jt.frictionloss = T.FRICTIONLOSS
            jt.damping[0] = T.VISCOUS_DAMPING
            a = spec.add_actuator(name=j, target=j)
            a.trntype = mujoco.mjtTrn.mjTRN_JOINT
            a.dyntype = mujoco.mjtDyn.mjDYN_NONE
            a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            a.gainprm[0] = T.KP
            a.biasprm[1] = -T.KP
            a.biasprm[2] = -T.KD
            a.inheritrange = 0.0
            a.ctrllimited = False
            a.forcelimited = True
            a.forcerange[:] = [-T.EFFORT_LIMIT, T.EFFORT_LIMIT]
        S = mujoco.mjtSensor
        for leg, geom in zip(T.FOOT_ORDER, T.TOE_GEOMS):
            for field, bit in (("found", 0), ("force", 1)):
                s = spec.add_sensor(name="{}_contact_{}".format(leg, field), type=S.mjSENS_CONTACT,
                                    objtype=mujoco.mjtObj.mjOBJ_GEOM, objname=geom)
                s.intprm[:3] = [1 << bit, 3, 1]   # netforce reduction, one slot
            spec.add_sensor(name="{}_pos".format(leg), type=S.mjSENS_FRAMEPOS,
                            objtype=mujoco.mjtObj.mjOBJ_SITE, objname=leg)
            spec.add_sensor(name="{}_linvel".format(leg), type=S.mjSENS_FRAMELINVEL,
                            objtype=mujoco.mjtObj.mjOBJ_SITE, objname=leg)
        for j in T.JOINT_NAMES:
            spec.add_sensor(name="{}_frc".format(j), type=S.mjSENS_ACTUATORFRC,
                            objtype=mujoco.mjtObj.mjOBJ_ACTUATOR, objname=j)
        m = spec.compile()
        o = m.opt
        o.timestep = T.PHYSICS_DT
        o.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        o.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL
        o.jacobian = mujoco.mjtJacobian.mjJAC_AUTO
        o.solver = mujoco.mjtSolver.mjSOL_NEWTON
        o.impratio = 1.0
        o.iterations = T.SOLVER_ITERATIONS
        o.tolerance = 1e-8
        o.ls_iterations = T.SOLVER_LS_ITERATIONS
        o.ls_tolerance = 0.01
        o.ccd_iterations = T.CCD_ITERATIONS
        o.gravity[:] = (0.0, 0.0, -9.81)
        self.model = m

        def adr(name: str) -> int:
            return int(m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)])

        self.jnt_ids = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
                                 for j in T.JOINT_NAMES])
        self.qadr = m.jnt_qposadr[self.jnt_ids]
        self.vadr = m.jnt_dofadr[self.jnt_ids]
        assert list(self.qadr) == list(range(7, 15)), "expected the free joint first, then the 8 legs"
        self.base_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, T.BASE_BODY)
        self.toe_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, g) for g in T.TOE_GEOMS]
        r3 = np.arange(3)
        self.found_idx = np.array([adr("{}_contact_found".format(leg)) for leg in T.FOOT_ORDER])
        self.force_idx = np.concatenate([adr("{}_contact_force".format(leg)) + r3 for leg in T.FOOT_ORDER])
        self.sitepos_idx = np.concatenate([adr("{}_pos".format(leg)) + r3 for leg in T.FOOT_ORDER])
        self.sitevel_idx = np.concatenate([adr("{}_linvel".format(leg)) + r3 for leg in T.FOOT_ORDER])
        self.frc_idx = np.array([adr("{}_frc".format(j)) for j in T.JOINT_NAMES])
        self.angmom_idx = adr("root_angmom") + r3

    def _startup_domain_randomization(self) -> None:
        """mjlab ``mode="startup"`` events: one draw per env, kept for the whole run."""
        N, nj, u = self.num_envs, self.num_actions, self.rng.uniform
        friction = u(*T.FOOT_FRICTION_RANGE, size=N)
        com = np.stack([u(lo, hi, size=N) for lo, hi in T.BASE_COM_RANGE], axis=1)
        self.encoder_bias = u(*T.ENCODER_BIAS_RANGE, size=(N, nj))
        kp_s = u(*T.KP_SCALE_RANGE, size=(N, nj))
        kd_s = u(*T.KD_SCALE_RANGE, size=(N, nj))
        eff_s = u(*T.EFFORT_SCALE_RANGE, size=(N, nj))
        arm_s = u(*T.ARMATURE_SCALE_RANGE, size=(N, nj))
        damp_s = u(*T.DAMPING_SCALE_RANGE, size=(N, nj))
        fric = u(*T.FRICTIONLOSS_RANGE, size=(N, nj))
        payload = u(*T.PAYLOAD_RANGE, size=N)
        d = mujoco.MjData(self.model)
        for i, m in enumerate(self.models):
            m.geom_friction[self.toe_ids, 0] = friction[i]
            m.body_ipos[self.base_id] += com[i]
            m.body_mass[self.base_id] += payload[i]
            m.actuator_gainprm[:, 0] = T.KP * kp_s[i]
            m.actuator_biasprm[:, 1] = -T.KP * kp_s[i]
            m.actuator_biasprm[:, 2] = -T.KD * kd_s[i]
            m.actuator_forcerange[:, 0] = -T.EFFORT_LIMIT * eff_s[i]
            m.actuator_forcerange[:, 1] = T.EFFORT_LIMIT * eff_s[i]
            m.dof_armature[self.vadr] = T.ARMATURE * arm_s[i]
            m.dof_damping[self.vadr] = T.VISCOUS_DAMPING * damp_s[i]
            m.dof_frictionloss[self.vadr] = fric[i]
            mujoco.mj_setConst(m, d)

    def _startup_terrain(self) -> None:
        m0 = self.model
        n = int(m0.hfield_nrow[0])
        size = float(m0.hfield_size[0, 0])
        gid = mujoco.mj_name2id(m0, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        self.ground_z = np.zeros(self.num_envs)
        for i, m in enumerate(self.models):
            h = _terrain_tile(self.rng, n, size)
            lo, span = float(h.min()), max(float(np.ptp(h)), 1e-4)
            m.hfield_size[0, 2] = span
            m.hfield_data[:] = ((h - lo) / span).ravel()
            m.geom_pos[gid, 2] = lo
            self.ground_z[i] = h[n // 2, n // 2]

    # ------------------------------------------------------------------ physics

    def _physics(self, target: np.ndarray) -> None:
        """The decimation loop: ``DECIMATION`` substeps in one ``rollout`` call.

        The target is constant within the control step, so the delayed
        command of every substep is known before stepping.
        """
        N, L = self.num_envs, T.DELAY_MAX_LAG + 1
        rows = np.arange(N)
        nu = self.num_actions
        for k in range(T.DECIMATION):
            # DelayBuffer: append, sample a lag per env in [min, max] every
            # physics step, clamp it to the history held since the last reset.
            self._delay_append(target)
            lag = self.rng.integers(T.DELAY_MIN_LAG, T.DELAY_MAX_LAG + 1, size=N)
            lag = np.minimum(lag, self._delay_len - 1)
            self._ctrl[:, k, :nu] = self._delay_hist[rows, (self._delay_ptr - lag) % L]
        if T.ERFI:
            noise = self.rng.uniform(-1.0, 1.0, size=(N, nu)) * T.ERFI_STEP_SCALE * T.EFFORT_LIMIT
            qfrc = np.zeros((N, self.model.nv))
            qfrc[:, self.vadr] = self.erfi_offset + noise
            self._ctrl[:, :, nu:] = qfrc[:, None, :]
        self.pool.rollout(self.models, self.datas, self.state, self._ctrl,
                          control_spec=self.control_spec, skip_checks=True,
                          nstep=T.DECIMATION, state=self._x_out, sensordata=self._sd_out)
        dt = T.PHYSICS_DT
        zero = np.float32(0.0)
        for k in range(T.DECIMATION):
            self.sim_time += np.float32(dt)
            elapsed = (self.sim_time - self.last_time)[:, None]
            self.last_time[:] = self.sim_time
            contact = self._sd_out[:, k, self.found_idx] > 0
            first_contact = (self.cur_air > 0) & contact
            first_detached = (self.cur_contact > 0) & ~contact
            self.last_air = np.where(first_contact, self.cur_air + elapsed, self.last_air)
            self.cur_air = np.where(~contact, self.cur_air + elapsed, zero)
            self.last_contact = np.where(first_detached, self.cur_contact + elapsed, self.last_contact)
            self.cur_contact = np.where(contact, self.cur_contact + elapsed, zero)
        v0 = 1 + self.model.nq
        prev_v = self._x_out[:, -2] if T.DECIMATION > 1 else self.state
        self.joint_acc = (self._x_out[:, -1, v0 + self.vadr] - prev_v[:, v0 + self.vadr]) / dt
        # The state the last substep started from: mjlab's derived quantities
        # (xpos, cvel, sites, sensors) refer to it until the next forward().
        self.prev_state = self._x_out[:, -2] if T.DECIMATION > 1 else self.state.copy()
        self.state[:] = self._x_out[:, -1]
        self.sd[:] = self._sd_out[:, -1]

    def _delay_append(self, values: np.ndarray) -> None:
        """One push into every env's delay line; a row emptied by a reset is
        backfilled with its first value (mjlab CircularBuffer)."""
        L = T.DELAY_MAX_LAG + 1
        self._delay_ptr = (self._delay_ptr + 1) % L
        self._delay_hist[:, self._delay_ptr] = values
        fresh = self._delay_len == 0
        if fresh.any():
            self._delay_hist[fresh] = values[fresh, None, :]
        self._delay_len = np.minimum(self._delay_len + 1, L)

    def _write_targets_after_reset(self, ids: np.ndarray, target: np.ndarray) -> None:
        """mjlab calls ``scene.write_data_to_sim()`` once more whenever any env
        was reset. That pushes every env's current joint target into the delay
        line an extra time -- and a reset zeroes the target, so the reset envs
        are commanded 0 rad (legs straight) until the first real target comes
        out of the delay, 4-7 physics steps into the new episode. It is part of
        the MDP the mjlab policies were trained on, so it is reproduced here."""
        values = target.copy()
        values[ids] = 0.0
        self._delay_append(values)

    def _forward_rows(self, ids: np.ndarray) -> None:
        """Fresh sensordata for rows whose state was just written (resets)."""
        d, nq, nv = self._fwd, self.model.nq, self.model.nv
        for i in ids:
            d.qpos[:] = self.state[i, 1:1 + nq]
            d.qvel[:] = self.state[i, 1 + nq:1 + nq + nv]
            mujoco.mj_forward(self.models[i], d)
            self.sd[i] = d.sensordata

    # ------------------------------------------------------------------ state views

    @property
    def qpos(self) -> np.ndarray:
        return self.state[:, 1:1 + self.model.nq]

    @property
    def qvel(self) -> np.ndarray:
        return self.state[:, 1 + self.model.nq:1 + self.model.nq + self.model.nv]

    def _root(self, state: np.ndarray | None = None) -> dict:
        """Base pose / velocity of ``state`` (default: the current state)."""
        x = self.state if state is None else state
        nq, nv = self.model.nq, self.model.nv
        q, v = x[:, 1:1 + nq], x[:, 1 + nq:1 + nq + nv]
        R = quat_to_mat(q[:, 3:7])
        ang_b = v[:, 3:6]
        return {
            "R": R,
            "pos": q[:, :3],
            "lin_vel_b": np.einsum("nji,nj->ni", R, v[:, :3]),
            "ang_vel_b": ang_b,
            "ang_vel_w": np.einsum("nij,nj->ni", R, ang_b),
            "projected_gravity": -R[:, 2, :],
        }

    # ------------------------------------------------------------------ commands / events

    def _resample_commands(self, ids: np.ndarray) -> None:
        n, u = len(ids), self.rng.uniform
        self.cmd_time_left[ids] = u(*T.CMD_RESAMPLING_TIME, size=n)
        cmd = np.stack([u(*self.cmd_lin_x, size=n), np.zeros(n), u(*self.cmd_ang_z, size=n)], axis=1)
        self.command[ids] = cmd
        self.is_standing[ids] = u(size=n) <= T.CMD_REL_STANDING_ENVS

    def _update_commands(self) -> None:
        if self.fixed_command is not None:
            self.command[:] = self.fixed_command
            return
        self.cmd_time_left -= T.STEP_DT
        ids = np.flatnonzero(self.cmd_time_left <= 0.0)
        if len(ids):
            self._resample_commands(ids)
        self.command[self.is_standing] = 0.0

    def _push(self) -> None:
        if self.play:
            return
        self.push_time_left -= T.STEP_DT
        ids = np.flatnonzero(self.push_time_left < 1e-6)
        if not len(ids):
            return
        self.push_time_left[ids] = self.rng.uniform(*T.PUSH_INTERVAL_S, size=len(ids))
        r = _ranges(T.PUSH_VELOCITY_RANGE)
        dv = self.rng.uniform(r[:, 0], r[:, 1], size=(len(ids), 6))
        R = quat_to_mat(self.qpos[ids, 3:7])
        v = self.qvel
        v[ids, :3] += dv[:, :3]
        v[ids, 3:6] += np.einsum("nji,nj->ni", R, dv[:, 3:])   # world -> body

    def _reset(self, ids: np.ndarray) -> None:
        n = len(ids)
        if not n:
            return
        if not self.play:
            for step, lin_x, ang_z in T.CMD_CURRICULUM:
                if self.common_step_counter > step:
                    self.cmd_lin_x, self.cmd_ang_z = lin_x, ang_z
        u = self.rng.uniform
        pose_r = _ranges(T.ROBUST_POSE_RANGE if self.robust else T.RESET_POSE_RANGE)
        vel_r = _ranges(T.ROBUST_VELOCITY_RANGE if self.robust else T.RESET_VELOCITY_RANGE)
        jp_r = T.ROBUST_JOINT_POS_RANGE if self.robust else T.RESET_JOINT_POS_RANGE
        jv_r = T.ROBUST_JOINT_VEL_RANGE if self.robust else T.RESET_JOINT_VEL_RANGE
        pose = u(pose_r[:, 0], pose_r[:, 1], size=(n, 6))
        vel = u(vel_r[:, 0], vel_r[:, 1], size=(n, 6))
        quat = quat_from_euler_xyz(pose[:, 3], pose[:, 4], pose[:, 5])
        q = np.zeros((n, self.model.nq))
        v = np.zeros((n, self.model.nv))
        q[:, :3] = pose[:, :3] + (0.0, 0.0, T.HOME_BASE_HEIGHT)
        if T.TERRAIN == "rough":
            q[:, 2] += self.ground_z[ids]
        q[:, 3:7] = quat
        jp = self.default_joint_pos + u(*jp_r, size=(n, self.num_actions))
        q[:, self.qadr] = np.clip(jp, self.soft_lo, self.soft_hi)
        v[:, :3] = vel[:, :3]
        v[:, 3:6] = np.einsum("nji,nj->ni", quat_to_mat(quat), vel[:, 3:])   # world -> body
        v[:, self.vadr] = u(*jv_r, size=(n, self.num_actions))
        self.state[ids] = 0.0
        self.state[ids, 1:1 + self.model.nq] = q
        self.state[ids, 1 + self.model.nq:1 + self.model.nq + self.model.nv] = v

        self.action[ids] = 0.0
        self.prev_action[ids] = 0.0
        self._delay_len[ids] = 0
        for arr in (self.cur_air, self.last_air, self.cur_contact, self.last_contact, self.vel_filtered,
                    self.sim_time, self.last_time):
            arr[ids] = 0.0
        self._resample_commands(ids)
        if T.ERFI:
            self.erfi_offset[ids] = (u(-1.0, 1.0, size=(n, self.num_actions))
                                     * T.ERFI_OFFSET_SCALE * T.EFFORT_LIMIT)
        self.episode_length[ids] = 0
        self._need_backfill[ids] = True

    # ------------------------------------------------------------------ rewards

    def _rewards(self, root: dict, terminated: np.ndarray) -> np.ndarray:
        N = self.num_envs
        cmd = self.command
        total_cmd = np.linalg.norm(cmd[:, :2], axis=1) + np.abs(cmd[:, 2])
        jp = self.qpos[:, self.qadr]
        jv = self.qvel[:, self.vadr]
        sd = self.sd
        found = sd[:, self.found_idx]
        in_contact = found > 0
        force = sd[:, self.force_idx].reshape(N, -1, 3)
        foot_z = sd[:, self.sitepos_idx].reshape(N, -1, 3)[..., 2]
        foot_vxy = np.linalg.norm(sd[:, self.sitevel_idx].reshape(N, -1, 3)[..., :2], axis=-1)
        act_frc = sd[:, self.frc_idx]
        v_b, w_b = root["lin_vel_b"], root["ang_vel_b"]
        first_contact = (self.cur_contact > 0.0) & (self.cur_contact < np.float32(T.STEP_DT + 1e-8))
        active = lambda thr: (total_cmd > thr).astype(np.float64)  # noqa: E731

        standing = total_cmd < T.POSE_WALKING_THRESHOLD
        running = total_cmd >= T.POSE_RUNNING_THRESHOLD
        std = np.where(standing[:, None], self.pose_std["standing"],
                       np.where(running[:, None], self.pose_std["running"], self.pose_std["walking"]))
        phase = (self.episode_length * T.STEP_DT / T.GAIT_PERIOD)[:, None]
        leg_phase = (phase + np.array(T.GAIT_OFFSET)[None]) % 1.0
        stance_ok = (leg_phase < T.FOOT_GAIT_THRESHOLD) == (self.cur_contact > 0)

        in_mode = np.where(self.cur_contact > 0, self.cur_contact, self.cur_air)
        single = np.mean(self.cur_contact > 0, axis=1) == 0.5
        mode_time = np.min(np.where(single[:, None], in_mode, 0.0), axis=1)
        air_rew = np.clip(T.FEET_AIR_TIME_THRESHOLD - np.abs(mode_time - T.FEET_AIR_TIME_THRESHOLD), 0.0, None)

        self.peak_heights = np.where(~in_contact, np.maximum(self.peak_heights, foot_z), self.peak_heights)
        swing_cost = np.sum(np.square(self.peak_heights / T.SWING_HEIGHT - 1.0) * first_contact, axis=1)
        self.peak_heights = np.where(first_contact, 0.0, self.peak_heights)

        alpha = T.STEP_DT / (T.VEL_FILTER_TIME + T.STEP_DT)
        self.vel_filtered = (1.0 - alpha) * self.vel_filtered + alpha * v_b[:, :2]
        excess = lambda x, lim: np.sum(np.square(np.clip(np.abs(x) - lim, 0.0, None)), axis=1)  # noqa: E731

        terms = {
            "track_linear_velocity": np.exp(
                -(np.sum(np.square(cmd[:, :2] - v_b[:, :2]), axis=1) + 2.0 * np.square(v_b[:, 2]))
                / T.TRACK_LIN_STD ** 2),
            "track_angular_velocity": np.exp(
                -(np.square(cmd[:, 2] - w_b[:, 2]) + 0.05 * np.sum(np.square(w_b[:, :2]), axis=1))
                / T.TRACK_ANG_STD ** 2),
            "body_orientation_l2": np.sum(np.square(root["projected_gravity"][:, :2]), axis=1),
            "pose": np.exp(-np.mean(np.square(jp - self.default_joint_pos) / np.square(std), axis=1)),
            "body_ang_vel": np.sum(np.square(root["ang_vel_w"][:, :2]), axis=1),
            "angular_momentum": np.sum(np.square(sd[:, self.angmom_idx]), axis=1),
            "is_terminated": terminated.astype(np.float64),
            "joint_acc_l2": np.sum(np.square(self.joint_acc), axis=1),
            "joint_pos_limits": np.sum(np.clip(self.soft_lo - jp, 0.0, None)
                                       + np.clip(jp - self.soft_hi, 0.0, None), axis=1),
            "action_rate_l2": np.sum(np.square(self.action - self.prev_action), axis=1),
            "foot_gait": np.mean(stance_ok, axis=1) * active(T.FOOT_GAIT_CMD_THRESHOLD),
            "foot_clearance": np.sum(np.abs(foot_z - T.SWING_HEIGHT) * foot_vxy, axis=1)
            * active(T.FOOT_CLEARANCE_CMD_THRESHOLD),
            "foot_slip": np.sum(np.square(foot_vxy) * in_contact, axis=1) * active(T.FOOT_SLIP_CMD_THRESHOLD),
            "soft_landing": np.sum(np.linalg.norm(force, axis=-1) * first_contact, axis=1)
            * active(T.SOFT_LANDING_CMD_THRESHOLD),
            "stand_still": np.sum(np.square(jp - self.default_joint_pos), axis=1)
            * (total_cmd <= T.STAND_STILL_CMD_THRESHOLD),
            "alive": (~terminated).astype(np.float64),
            "feet_air_time": air_rew * active(T.FEET_AIR_TIME_CMD_THRESHOLD),
            "feet_swing_height": swing_cost * active(T.FEET_SWING_CMD_THRESHOLD),
            "joint_vel_l2": np.sum(np.square(jv), axis=1),
            "joint_torques_l2": np.sum(np.square(act_frc), axis=1),
            "torque_excess": excess(act_frc - T.VISCOUS_DAMPING * jv, T.TORQUE_EXCESS_LIMIT),
            "speed_excess": excess(jv, T.SPEED_EXCESS_LIMIT),
            "base_height": np.square(np.clip(T.HOME_BASE_HEIGHT - root["pos"][:, 2], 0.0, None)),
            "track_lin_vel_filtered": np.exp(
                -np.sum(np.square(cmd[:, :2] - self.vel_filtered), axis=1) / T.TRACK_LIN_STD ** 2),
            "base_vel_oscillation": np.sum(np.square(v_b[:, :2] - self.vel_filtered), axis=1),
            "foot_air_time_excess": np.sum(np.clip(self.cur_air - T.FOOT_AIR_TIME_EXCESS_LIMIT, 0.0, None),
                                           axis=1),
        }
        # Per-term value x weight, before the dt scaling (mjlab's _step_reward).
        self.step_reward = {name: value * T.REWARD_WEIGHTS[name] for name, value in terms.items()}
        reward = np.zeros(N)
        for name, value in terms.items():
            v = np.nan_to_num(value * T.REWARD_WEIGHTS[name] * T.STEP_DT, nan=0.0, posinf=0.0, neginf=0.0)
            reward += v
            self.episode_sums[name] += v
        self.extras["log"]["Metrics/filtered_vx_mean"] = float(self.vel_filtered[:, 0].mean())
        return reward

    # ------------------------------------------------------------------ observations

    def _build_obs_layout(self) -> None:
        nj, nf = self.num_actions, len(T.FOOT_ORDER)
        dims = {"command": 3, "phase": 2, "joint_pos": nj, "joint_vel": nj, "actions": nj,
                "base_ang_vel": 3, "projected_gravity": 3, "base_lin_vel": 3,
                "foot_height": nf, "foot_air_time": nf, "foot_contact": nf, "foot_contact_forces": 3 * nf}
        self._actor_step_dim = sum(dims[t] for t in T.ACTOR_TERMS)
        # History is kept per step as (H, step_dim); the network wants each term's
        # history contiguous, oldest first (mjlab's flattened CircularBuffer).
        order, off = [], 0
        for t in T.ACTOR_TERMS:
            for h in range(T.ACTOR_HISTORY):
                order.extend(h * self._actor_step_dim + off + np.arange(dims[t]))
            off += dims[t]
        self._actor_perm = np.array(order)
        self.actor_dim = self._actor_step_dim * T.ACTOR_HISTORY
        self.critic_dim = sum(dims[t] for t in T.CRITIC_TERMS)

    def _observe(self, reset_ids: np.ndarray) -> None:
        if len(reset_ids):
            self._forward_rows(reset_ids)
        N = self.num_envs
        root = self._root()
        jp = self.qpos[:, self.qadr] - self.default_joint_pos
        jv = self.qvel[:, self.vadr].copy()
        g = (self.episode_length * T.STEP_DT) % T.GAIT_PERIOD / T.GAIT_PERIOD
        phase = np.stack([np.sin(2 * math.pi * g), np.cos(2 * math.pi * g)], axis=1)
        phase[np.linalg.norm(self.command, axis=1) < T.PHASE_ZERO_BELOW] = 0.0
        sd = self.sd
        force = sd[:, self.force_idx]
        terms = {
            "command": self.command, "phase": phase, "joint_pos": jp, "joint_vel": jv,
            "actions": self.action,
            "base_ang_vel": root["ang_vel_b"], "projected_gravity": root["projected_gravity"],
            "base_lin_vel": root["lin_vel_b"],
            "foot_height": sd[:, self.sitepos_idx].reshape(N, -1, 3)[..., 2],
            "foot_air_time": self.cur_air,
            "foot_contact": (sd[:, self.found_idx] > 0).astype(np.float64),
            "foot_contact_forces": np.sign(force) * np.log1p(np.abs(force)),
        }
        critic = np.concatenate([terms[t] for t in T.CRITIC_TERMS], axis=1)
        if not self.play:
            terms["joint_pos"] = jp + self.rng.uniform(-T.JOINT_POS_NOISE, T.JOINT_POS_NOISE, size=jp.shape)
            terms["joint_vel"] = jv + self.rng.uniform(-T.JOINT_VEL_NOISE, T.JOINT_VEL_NOISE, size=jv.shape)
        step = np.concatenate([terms[t] for t in T.ACTOR_TERMS], axis=1)
        self._actor_hist[:, :-1] = self._actor_hist[:, 1:]
        self._actor_hist[:, -1] = step
        fill = self._need_backfill
        if fill.any():
            self._actor_hist[fill] = step[fill, None, :]
            fill[:] = False
        actor = self._actor_hist.reshape(N, -1)[:, self._actor_perm]
        self.obs = TensorDict({
            "actor": torch.from_numpy(actor.astype(np.float32)),
            "critic": torch.from_numpy(critic.astype(np.float32)),
        }, batch_size=[N])

    # ------------------------------------------------------------------ VecEnv

    @property
    def episode_length_buf(self) -> torch.Tensor:
        return torch.from_numpy(self.episode_length.copy())

    @episode_length_buf.setter
    def episode_length_buf(self, value: torch.Tensor) -> None:
        self.episode_length[:] = value.cpu().numpy()

    def get_observations(self) -> TensorDict:
        return self.obs

    def step(self, actions: torch.Tensor) -> tuple[TensorDict, torch.Tensor, torch.Tensor, dict]:
        self.extras = {"log": {}}
        self.prev_action[:] = self.action
        self.action[:] = actions.detach().cpu().numpy()
        target = self.default_joint_pos + T.ACTION_SCALE * self.action - self.encoder_bias
        self._physics(target)
        self.episode_length += 1
        self.common_step_counter += 1

        root = self._root(self.prev_state)
        time_out = self.episode_length >= self.max_episode_length
        fell = np.arccos(np.clip(-root["projected_gravity"][:, 2], -1.0, 1.0)) > T.FELL_OVER_ANGLE
        low = root["pos"][:, 2] < (T.MIN_BASE_HEIGHT + (self.ground_z if T.TERRAIN == "rough" else 0.0))
        terminated = fell | low
        reward = self._rewards(root, terminated)

        done = terminated | time_out
        ids = np.flatnonzero(done)
        if len(ids):
            log = self.extras["log"]
            for name, s in self.episode_sums.items():
                log["Episode_Reward/" + name] = float(s[ids].mean() / T.EPISODE_LENGTH_S)
                s[ids] = 0.0
            log["Episode_Termination/time_out"] = int(time_out[ids].sum())
            log["Episode_Termination/fell_over"] = int(fell[ids].sum())
            log["Episode_Termination/low_base"] = int((low & ~fell)[ids].sum())
            log["Curriculum/lin_vel_x_max"] = float(self.cmd_lin_x[1])
            self._reset(ids)
            self._write_targets_after_reset(ids, target)
        self._update_commands()
        self._push()
        self._observe(ids)
        self.extras["time_outs"] = torch.from_numpy(time_out)
        return (self.obs, torch.from_numpy(reward.astype(np.float32)),
                torch.from_numpy(done.astype(np.int64)), self.extras)

    def set_command(self, vx: float, wz: float) -> None:
        """Pin every env to one command and restart them all (evaluation)."""
        self.fixed_command = np.array([vx, 0.0, wz])
        ids = np.arange(self.num_envs)
        self._reset(ids)
        self._write_targets_after_reset(ids, np.zeros((self.num_envs, self.num_actions)))
        self._update_commands()
        self._observe(ids)

    def close(self) -> None:
        self.pool.close()


def mujoco_threads() -> int:
    """Rollout pool size: every logical CPU this process may use."""
    try:
        return len(os.sched_getaffinity(0))
    except AttributeError:   # macOS
        return os.cpu_count() or 1
