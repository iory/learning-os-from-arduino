"""ArduinoQuad-Walk / -Robust as a batched Genesis environment.

The same MDP as ``../cpu/quad_env.py`` -- every number comes from
``../cpu/task.py`` -- with the physics on the GPU in Genesis instead of
``mujoco.rollout``. Commands, observations, rewards and resets are torch on the
same device, term by term as in the CPU env:

  action -> target = home + 0.25 a - encoder_bias
  4 physics substeps, each: push the target into the delay line, read it back
      4..7 physics steps late, Genesis PD + back-EMF damping + effort limit,
      update foot air/contact time from the toe contact forces
  terminations -> rewards (x dt) -> reset finished envs -> resample commands
  -> random pushes -> observations (noise, then a 3-step history per term)

The policy that comes out is meant to be run in MuJoCo (``sim2sim.py``, and the
browser simulator, which is MuJoCo WASM), so only what the policy sees and is
rewarded for is copied from the CPU env. Two of its mjlab-reproducing details
are left out on purpose, because they belong to mjlab's implementation and
not to the robot:

* Rewards and terminations see the state after the last substep, not the
  state one substep before it.
* A reset does not push a 0 rad target into the delay line; the new episode
  starts from the home targets.

Not supported here: ``ARDUINO_QUAD_TERRAIN=rough`` and ``ARDUINO_QUAD_ERFI=1``.
"""

import math
import sys
import tempfile
from pathlib import Path

import genesis as gs
import mujoco
import numpy as np
import torch
from rsl_rl.env import VecEnv
from tensordict import TensorDict

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "cpu"))
import task as T  # noqa: E402

_POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")
# Genesis takes the larger of the two friction coefficients of a contact
# (MuJoCo picks the toe's through its priority), so the floor gets the smallest
# coefficient Genesis accepts and every contact uses the toe's own.
FLOOR_FRICTION = 0.01


def _ranges(d: dict, device) -> torch.Tensor:
    return torch.tensor([d.get(k, (0.0, 0.0)) for k in _POSE_KEYS], dtype=torch.float32, device=device)


def quat_to_mat(q: torch.Tensor) -> torch.Tensor:
    """(..., 4) wxyz -> (..., 3, 3) body-to-world rotation."""
    w, x, y, z = q.unbind(-1)
    return torch.stack([
        1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
        2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
        2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
    ], dim=-1).reshape(q.shape[:-1] + (3, 3))


def quat_from_euler_xyz(roll: torch.Tensor, pitch: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
    """Same convention as mjlab's ``quat_from_euler_xyz`` (R = Rz Ry Rx), wxyz."""
    cr, sr = torch.cos(roll * 0.5), torch.sin(roll * 0.5)
    cp, sp = torch.cos(pitch * 0.5), torch.sin(pitch * 0.5)
    cy, sy = torch.cos(yaw * 0.5), torch.sin(yaw * 0.5)
    return torch.stack([
        cy * cr * cp + sy * sr * sp,
        cy * sr * cp - sy * cr * sp,
        cy * cr * sp + sy * sr * cp,
        sy * cr * cp - cy * sr * sp,
    ], dim=-1)


def physics_mjcf(path: Path) -> dict:
    """Write the MJCF Genesis simulates and return the toe sites.

    The same reduction as the CPU env's ``_build_model``: only the four toe
    spheres collide, meshes are dropped (the MJCF has explicit inertials), and
    the joints get the armature, stiction and back-EMF damping of task.py. The
    floor is added in Genesis, so the MJCF's own plane is dropped too.

    Returns
    -------
    dict
        ``{leg: (foot body name, site position in that body)}`` for FOOT_ORDER.
    """
    spec = mujoco.MjSpec.from_file(str(T.MJCF))
    for item in list(spec.actuators) + list(spec.keys) + list(spec.sensors):
        spec.delete(item)
    toes = set(T.TOE_GEOMS)
    for g in list(spec.geoms):
        if g.name not in toes:
            spec.delete(g)
    for mesh in list(spec.meshes):
        spec.delete(mesh)
    for name in T.TOE_GEOMS:
        g = spec.geom(name)
        g.contype, g.conaffinity, g.condim = 1, 1, T.TOE_CONDIM
        g.friction[0] = T.TOE_FRICTION
        g.solimp[:3] = T.TOE_SOLIMP
    for j in T.JOINT_NAMES:
        jt = spec.joint(j)
        jt.armature = T.ARMATURE
        jt.frictionloss = T.FRICTIONLOSS
        jt.damping[0] = T.VISCOUS_DAMPING
    model = spec.compile()
    path.write_text(spec.to_xml())
    sites = {}
    for leg in T.TOE_SITES:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, leg)
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.site_bodyid[sid]))
        sites[leg] = (body, model.site_pos[sid].copy())
    return sites


_GS_READY = False


def init_genesis(device: str) -> None:
    """``gs.init`` once per process (it refuses a second call)."""
    global _GS_READY
    if _GS_READY:
        return
    backend = gs.cpu if device == "cpu" else gs.gpu
    gs.init(backend=backend, precision="32", logging_level="warning")
    _GS_READY = True


class GenesisQuadEnv(VecEnv):
    """rsl_rl ``VecEnv`` over ``num_envs`` Genesis worlds of the quadruped.

    Parameters
    ----------
    num_envs : int
        Number of parallel environments.
    robust : bool
        ``True`` for ArduinoQuad-Robust (off-nominal resets), else ArduinoQuad-Walk.
    play : bool
        No observation noise, no pushes, no command curriculum (full range)
        and no time limit, as in the CPU env.
    domain_rand : bool
        Apply the startup domain randomization of task.py.
    device : str
        ``"gpu"`` (CUDA, or Metal on a Mac, with torch on MPS) or ``"cpu"``.
    seed : int
        Seed for everything this env samples.
    """

    def __init__(self, num_envs: int, robust: bool = False, play: bool = False,
                 domain_rand: bool = True, device: str = "gpu", seed: int = 0) -> None:
        if T.TERRAIN != "plane":
            raise NotImplementedError("the Genesis trainer only has the flat floor (ARDUINO_QUAD_TERRAIN=plane)")
        if T.ERFI:
            raise NotImplementedError("the Genesis trainer has no ERFI (unset ARDUINO_QUAD_ERFI)")
        init_genesis(device)
        self.num_envs = num_envs
        self.num_actions = len(T.JOINT_NAMES)
        self.device = gs.device
        self.robust = robust
        self.play = play
        self.gen = torch.Generator(device=self.device)
        self.gen.manual_seed(seed)
        self.max_episode_length = (
            2 ** 62 if play else int(math.ceil(T.EPISODE_LENGTH_S / T.STEP_DT)))
        self.cfg = {"task": "ArduinoQuad-Robust" if robust else "ArduinoQuad-Walk",
                    "num_envs": num_envs, "play": play, "domain_rand": domain_rand,
                    "trainer": "genesis"}

        self._build_scene()
        N, nj, dev = num_envs, self.num_actions, self.device
        self.default_joint_pos = torch.tensor(T.DEFAULT_JOINT_POS, dtype=torch.float32, device=dev)
        lo, hi = self.robot.get_dofs_limit(self.dofs)
        lo, hi = lo.reshape(-1, nj)[0], hi.reshape(-1, nj)[0]
        mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo) * T.SOFT_JOINT_POS_LIMIT_FACTOR
        self.soft_lo, self.soft_hi = mid - half, mid + half
        hip = torch.tensor(["hip" in j for j in T.JOINT_NAMES], device=dev)
        self.pose_std = {
            name: torch.where(hip, d["hip"], d["knee"]).float()
            for name, d in (("standing", T.STD_STANDING), ("walking", T.STD_WALKING),
                            ("running", T.STD_RUNNING))}

        self.encoder_bias = torch.zeros(N, nj, device=dev)
        if domain_rand:
            self._startup_domain_randomization()
        self._cache_inertia()

        self.action = torch.zeros(N, nj, device=dev)
        self.prev_action = torch.zeros(N, nj, device=dev)
        L = T.DELAY_MAX_LAG + 1
        self._delay_hist = torch.zeros(N, L, nj, device=dev)
        self._delay_len = torch.zeros(N, dtype=torch.long, device=dev)
        self._delay_ptr = 0
        self.command = torch.zeros(N, 3, device=dev)
        self.cmd_time_left = torch.zeros(N, device=dev)
        self.is_standing = torch.zeros(N, dtype=torch.bool, device=dev)
        self.cmd_lin_x, self.cmd_ang_z = T.CMD_CURRICULUM[0][1], T.CMD_CURRICULUM[0][2]
        if play:
            self.cmd_lin_x, self.cmd_ang_z = T.CMD_LIN_X, T.CMD_ANG_Z
        self.fixed_command: torch.Tensor | None = None
        self.push_time_left = self._uniform(*T.PUSH_INTERVAL_S, (N,))
        self.episode_length_buf = torch.zeros(N, dtype=torch.long, device=dev)
        self.common_step_counter = 0
        nf = len(T.FOOT_ORDER)
        self.cur_air = torch.zeros(N, nf, device=dev)
        self.last_air = torch.zeros(N, nf, device=dev)
        self.cur_contact = torch.zeros(N, nf, device=dev)
        self.last_contact = torch.zeros(N, nf, device=dev)
        self.peak_heights = torch.zeros(N, nf, device=dev)
        self.vel_filtered = torch.zeros(N, 2, device=dev)
        self.joint_acc = torch.zeros(N, nj, device=dev)
        self.episode_sums = {k: torch.zeros(N, device=dev) for k in T.REWARD_WEIGHTS}

        self._build_obs_layout()
        self._need_backfill = torch.ones(N, dtype=torch.bool, device=dev)
        self._actor_hist = torch.zeros(N, T.ACTOR_HISTORY, self._actor_step_dim, device=dev)
        self.extras: dict = {"log": {}}
        all_ids = torch.arange(N, device=dev)
        self._reset(all_ids)
        self._read_state()
        self._observe()

    # ------------------------------------------------------------------ scene

    def _build_scene(self) -> None:
        self._tmp = tempfile.TemporaryDirectory(prefix="arduino_quad_genesis_")
        xml = Path(self._tmp.name) / "arduino_quad_physics.xml"
        sites = physics_mjcf(xml)
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=T.PHYSICS_DT, substeps=1, gravity=(0.0, 0.0, -9.81)),
            rigid_options=gs.options.RigidOptions(
                integrator=gs.integrator.implicitfast,
                constraint_solver=gs.constraint_solver.Newton,
                friction_cone=gs.friction_cone.pyramidal,
                iterations=T.SOLVER_ITERATIONS,
                ls_iterations=T.SOLVER_LS_ITERATIONS,
                enable_self_collision=False,
                batch_dofs_info=True,
                batch_links_info=True,
            ),
            show_viewer=False,
        )
        self.scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(friction=FLOOR_FRICTION))
        self.robot = self.scene.add_entity(gs.morphs.MJCF(file=str(xml), pos=(0.0, 0.0, T.HOME_BASE_HEIGHT)))
        self.scene.build(n_envs=self.num_envs)

        r = self.robot
        self.dofs = [r.get_joint(j).dofs_idx_local[0] for j in T.JOINT_NAMES]
        self.qs = [r.get_joint(j).qs_idx_local[0] for j in T.JOINT_NAMES]
        assert self.qs == list(range(7, 15)), "expected the free joint first, then the 8 legs"
        self.base_dofs = list(range(6))
        names = [link.name for link in r.links]
        self.base_link = names.index(T.BASE_BODY)
        self.foot_links = [names.index(sites[leg][0]) for leg in T.FOOT_ORDER]
        self.site_off = torch.tensor(np.stack([sites[leg][1] for leg in T.FOOT_ORDER]),
                                     dtype=torch.float32, device=gs.device)
        r.set_dofs_kp(torch.full((self.num_actions,), T.KP), self.dofs)
        r.set_dofs_kv(torch.full((self.num_actions,), T.KD), self.dofs)
        r.set_dofs_force_range(torch.full((self.num_actions,), -T.EFFORT_LIMIT),
                               torch.full((self.num_actions,), T.EFFORT_LIMIT), self.dofs)

    def _uniform(self, lo, hi, shape) -> torch.Tensor:
        lo = torch.as_tensor(lo, dtype=torch.float32, device=self.device)
        hi = torch.as_tensor(hi, dtype=torch.float32, device=self.device)
        return lo + (hi - lo) * torch.rand(shape, generator=self.gen, device=self.device)

    def _startup_domain_randomization(self) -> None:
        """One draw per env, kept for the whole run (mjlab ``mode="startup"``)."""
        N, nj, u, r = self.num_envs, self.num_actions, self._uniform, self.robot
        friction = u(*T.FOOT_FRICTION_RANGE, (N, 1)).expand(N, len(self.foot_links))
        r.set_friction_ratio(friction / T.TOE_FRICTION, self.foot_links)
        com = torch.stack([u(lo, hi, (N,)) for lo, hi in T.BASE_COM_RANGE], dim=1)
        com0 = r.get_links_COM([self.base_link]).reshape(N, 1, 3)
        r.set_links_COM(com0 + com[:, None, :], [self.base_link])
        mass0 = r.get_links_mass([self.base_link]).reshape(N, 1)
        r.set_links_mass(mass0 + u(*T.PAYLOAD_RANGE, (N, 1)), [self.base_link])
        self.encoder_bias = u(*T.ENCODER_BIAS_RANGE, (N, nj))
        r.set_dofs_kp(T.KP * u(*T.KP_SCALE_RANGE, (N, nj)), self.dofs)
        r.set_dofs_kv(T.KD * u(*T.KD_SCALE_RANGE, (N, nj)), self.dofs)
        effort = T.EFFORT_LIMIT * u(*T.EFFORT_SCALE_RANGE, (N, nj))
        r.set_dofs_force_range(-effort, effort, self.dofs)
        r.set_dofs_armature(T.ARMATURE * u(*T.ARMATURE_SCALE_RANGE, (N, nj)), self.dofs)
        r.set_dofs_damping(T.VISCOUS_DAMPING * u(*T.DAMPING_SCALE_RANGE, (N, nj)), self.dofs)
        r.set_dofs_frictionloss(u(*T.FRICTIONLOSS_RANGE, (N, nj)), self.dofs)

    def _cache_inertia(self) -> None:
        """Per-link mass, COM offset and inertia in the link frame, for the
        whole-robot angular momentum (MuJoCo's ``subtreeangmom`` of base_link)."""
        r, N = self.robot, self.num_envs
        self.link_mass = r.get_links_mass().reshape(N, -1)
        self.link_com = r.get_links_COM().reshape(N, -1, 3)
        principal = r.get_links_inertia().reshape(N, -1, 3, 3)
        rot = quat_to_mat(torch.tensor(np.stack([link.desc.inertial_quat for link in r.links]),
                                       dtype=torch.float32, device=self.device))
        self.link_inertia = rot @ principal @ rot.transpose(-1, -2)

    # ------------------------------------------------------------------ physics

    def _physics(self, target: torch.Tensor) -> None:
        N, L = self.num_envs, T.DELAY_MAX_LAG + 1
        rows = torch.arange(N, device=self.device)
        for _ in range(T.DECIMATION):
            self._delay_append(target)
            lag = torch.randint(T.DELAY_MIN_LAG, T.DELAY_MAX_LAG + 1, (N,), generator=self.gen, device=self.device)
            lag = torch.minimum(lag, self._delay_len - 1)
            ctrl = self._delay_hist[rows, (self._delay_ptr - lag) % L]
            self.robot.control_dofs_position(ctrl, self.dofs)
            prev_jv = self.robot.get_dofs_velocity(self.dofs)
            self.scene.step()
            contact = self._toe_force().norm(dim=-1) > 0.0
            dt = T.PHYSICS_DT
            first_contact = (self.cur_air > 0) & contact
            first_detached = (self.cur_contact > 0) & ~contact
            self.last_air = torch.where(first_contact, self.cur_air + dt, self.last_air)
            self.cur_air = torch.where(~contact, self.cur_air + dt, torch.zeros_like(self.cur_air))
            self.last_contact = torch.where(first_detached, self.cur_contact + dt, self.last_contact)
            self.cur_contact = torch.where(contact, self.cur_contact + dt, torch.zeros_like(self.cur_contact))
        self.joint_acc = (self.robot.get_dofs_velocity(self.dofs) - prev_jv) / T.PHYSICS_DT

    def _toe_force(self) -> torch.Tensor:
        """(N, 4, 3) net contact force on each foot link (only the toe collides)."""
        return self.robot.get_links_net_contact_force()[:, self.foot_links]

    def _delay_append(self, values: torch.Tensor) -> None:
        """One push into every env's delay line; a row emptied by a reset is
        backfilled with its first value (mjlab CircularBuffer)."""
        L = T.DELAY_MAX_LAG + 1
        self._delay_ptr = (self._delay_ptr + 1) % L
        self._delay_hist[:, self._delay_ptr] = values
        fresh = self._delay_len == 0
        self._delay_hist[fresh] = values[fresh, None, :]
        self._delay_len = torch.clamp(self._delay_len + 1, max=L)

    # ------------------------------------------------------------------ state

    def _read_state(self) -> None:
        """Everything rewards and observations need, from the current state."""
        r, N = self.robot, self.num_envs
        pos, quat = r.get_pos(), r.get_quat()
        R = quat_to_mat(quat)
        base_v = r.get_dofs_velocity(self.base_dofs)
        ang_b = base_v[:, 3:6]   # the free joint's angular dofs are in the body frame
        self.root = {
            "R": R,
            "pos": pos,
            "quat": quat,
            "lin_vel_b": torch.einsum("nji,nj->ni", R, base_v[:, :3]),
            "ang_vel_b": ang_b,
            "ang_vel_w": torch.einsum("nij,nj->ni", R, ang_b),
            "projected_gravity": -R[:, 2, :],
        }
        self.jp = r.get_dofs_position(self.dofs)
        self.jv = r.get_dofs_velocity(self.dofs)
        links_pos = r.get_links_pos()
        links_R = quat_to_mat(r.get_links_quat())
        links_v = r.get_links_vel()
        links_w = r.get_links_ang()
        fR = links_R[:, self.foot_links]
        off = torch.einsum("nfij,fj->nfi", fR, self.site_off)
        self.foot_pos = links_pos[:, self.foot_links] + off
        self.foot_vel = links_v[:, self.foot_links] + torch.cross(links_w[:, self.foot_links], off, dim=-1)
        self.toe_force = self._toe_force()
        self.in_contact = self.toe_force.norm(dim=-1) > 0.0
        self.act_frc = r.get_dofs_control_force(self.dofs)
        # Angular momentum of the whole robot about its centre of mass.
        com_w = links_pos + torch.einsum("nlij,nlj->nli", links_R, self.link_com)
        v_com = r.get_links_vel(ref=gs.link_ref_frame.link_COM)
        m = self.link_mass[..., None]
        c = (m * com_w).sum(1, keepdim=True) / m.sum(1, keepdim=True)
        vc = (m * v_com).sum(1, keepdim=True) / m.sum(1, keepdim=True)
        orbital = torch.cross(com_w - c, m * (v_com - vc), dim=-1).sum(1)
        I_w = links_R @ self.link_inertia @ links_R.transpose(-1, -2)
        spin = torch.einsum("nlij,nlj->ni", I_w, links_w)
        self.angmom = orbital + spin

    # ------------------------------------------------------------------ commands / events

    def _resample_commands(self, ids: torch.Tensor) -> None:
        n, u = len(ids), self._uniform
        self.cmd_time_left[ids] = u(*T.CMD_RESAMPLING_TIME, (n,))
        cmd = torch.stack([u(*self.cmd_lin_x, (n,)), torch.zeros(n, device=self.device),
                           u(*self.cmd_ang_z, (n,))], dim=1)
        self.command[ids] = cmd
        self.is_standing[ids] = torch.rand(n, generator=self.gen, device=self.device) <= T.CMD_REL_STANDING_ENVS

    def _update_commands(self) -> None:
        if self.fixed_command is not None:
            self.command[:] = self.fixed_command
            return
        self.cmd_time_left -= T.STEP_DT
        ids = torch.nonzero(self.cmd_time_left <= 0.0).flatten()
        if len(ids):
            self._resample_commands(ids)
        self.command[self.is_standing] = 0.0

    def _push(self) -> None:
        if self.play:
            return
        self.push_time_left -= T.STEP_DT
        ids = torch.nonzero(self.push_time_left < 1e-6).flatten()
        if not len(ids):
            return
        self.push_time_left[ids] = self._uniform(*T.PUSH_INTERVAL_S, (len(ids),))
        r = _ranges(T.PUSH_VELOCITY_RANGE, self.device)
        dv = self._uniform(r[:, 0], r[:, 1], (len(ids), 6))
        R = self.root["R"][ids]
        v = self.robot.get_dofs_velocity(self.base_dofs, envs_idx=ids)
        v[:, :3] += dv[:, :3]
        v[:, 3:6] += torch.einsum("nji,nj->ni", R, dv[:, 3:])   # world -> body
        self.robot.set_dofs_velocity(v, self.base_dofs, envs_idx=ids)

    def _reset(self, ids: torch.Tensor) -> None:
        n = len(ids)
        if not n:
            return
        if not self.play:
            for step, lin_x, ang_z in T.CMD_CURRICULUM:
                if self.common_step_counter > step:
                    self.cmd_lin_x, self.cmd_ang_z = lin_x, ang_z
        u, dev = self._uniform, self.device
        pose_r = _ranges(T.ROBUST_POSE_RANGE if self.robust else T.RESET_POSE_RANGE, dev)
        vel_r = _ranges(T.ROBUST_VELOCITY_RANGE if self.robust else T.RESET_VELOCITY_RANGE, dev)
        jp_r = T.ROBUST_JOINT_POS_RANGE if self.robust else T.RESET_JOINT_POS_RANGE
        jv_r = T.ROBUST_JOINT_VEL_RANGE if self.robust else T.RESET_JOINT_VEL_RANGE
        pose = u(pose_r[:, 0], pose_r[:, 1], (n, 6))
        vel = u(vel_r[:, 0], vel_r[:, 1], (n, 6))
        quat = quat_from_euler_xyz(pose[:, 3], pose[:, 4], pose[:, 5])
        q = torch.zeros(n, 15, device=dev)
        q[:, :3] = pose[:, :3]
        q[:, 2] += T.HOME_BASE_HEIGHT
        q[:, 3:7] = quat
        jp = self.default_joint_pos + u(*jp_r, (n, self.num_actions))
        q[:, 7:] = torch.clamp(jp, self.soft_lo, self.soft_hi)
        self.robot.set_qpos(q, envs_idx=ids, zero_velocity=True)
        v = torch.zeros(n, 14, device=dev)
        v[:, :3] = vel[:, :3]
        v[:, 3:6] = torch.einsum("nji,nj->ni", quat_to_mat(quat), vel[:, 3:])   # world -> body
        v[:, 6:] = u(*jv_r, (n, self.num_actions))
        self.robot.set_dofs_velocity(v, envs_idx=ids)

        self.action[ids] = 0.0
        self.prev_action[ids] = 0.0
        self._delay_len[ids] = 0
        for buf in (self.cur_air, self.last_air, self.cur_contact, self.last_contact, self.vel_filtered,
                    self.peak_heights):
            buf[ids] = 0.0
        self._resample_commands(ids)
        self.episode_length_buf[ids] = 0
        self._need_backfill[ids] = True

    # ------------------------------------------------------------------ rewards

    def _rewards(self, terminated: torch.Tensor) -> torch.Tensor:
        root, cmd = self.root, self.command
        total_cmd = torch.linalg.norm(cmd[:, :2], dim=1) + torch.abs(cmd[:, 2])
        jp, jv = self.jp, self.jv
        in_contact = self.in_contact
        foot_z = self.foot_pos[..., 2]
        foot_vxy = torch.linalg.norm(self.foot_vel[..., :2], dim=-1)
        act_frc = self.act_frc
        v_b, w_b = root["lin_vel_b"], root["ang_vel_b"]
        first_contact = (self.cur_contact > 0.0) & (self.cur_contact < T.STEP_DT + 1e-8)

        def active(thr: float) -> torch.Tensor:
            return (total_cmd > thr).float()

        standing = (total_cmd < T.POSE_WALKING_THRESHOLD)[:, None]
        running = (total_cmd >= T.POSE_RUNNING_THRESHOLD)[:, None]
        std = torch.where(standing, self.pose_std["standing"],
                          torch.where(running, self.pose_std["running"], self.pose_std["walking"]))
        phase = (self.episode_length_buf * T.STEP_DT / T.GAIT_PERIOD)[:, None]
        leg_phase = (phase + torch.tensor(T.GAIT_OFFSET, device=self.device)[None]) % 1.0
        stance_ok = (leg_phase < T.FOOT_GAIT_THRESHOLD) == (self.cur_contact > 0)

        in_mode = torch.where(self.cur_contact > 0, self.cur_contact, self.cur_air)
        single = (self.cur_contact > 0).float().mean(dim=1) == 0.5
        mode_time = torch.where(single[:, None], in_mode, torch.zeros_like(in_mode)).min(dim=1).values
        air_rew = torch.clamp(T.FEET_AIR_TIME_THRESHOLD - torch.abs(mode_time - T.FEET_AIR_TIME_THRESHOLD), min=0.0)

        self.peak_heights = torch.where(~in_contact, torch.maximum(self.peak_heights, foot_z), self.peak_heights)
        swing_cost = (torch.square(self.peak_heights / T.SWING_HEIGHT - 1.0) * first_contact).sum(dim=1)
        self.peak_heights = torch.where(first_contact, torch.zeros_like(self.peak_heights), self.peak_heights)

        alpha = T.STEP_DT / (T.VEL_FILTER_TIME + T.STEP_DT)
        self.vel_filtered = (1.0 - alpha) * self.vel_filtered + alpha * v_b[:, :2]

        def excess(x: torch.Tensor, lim: float) -> torch.Tensor:
            return torch.square(torch.clamp(torch.abs(x) - lim, min=0.0)).sum(dim=1)

        sq = torch.square
        terms = {
            "track_linear_velocity": torch.exp(
                -(sq(cmd[:, :2] - v_b[:, :2]).sum(1) + 2.0 * sq(v_b[:, 2])) / T.TRACK_LIN_STD ** 2),
            "track_angular_velocity": torch.exp(
                -(sq(cmd[:, 2] - w_b[:, 2]) + 0.05 * sq(w_b[:, :2]).sum(1)) / T.TRACK_ANG_STD ** 2),
            "body_orientation_l2": sq(root["projected_gravity"][:, :2]).sum(1),
            "pose": torch.exp(-(sq(jp - self.default_joint_pos) / sq(std)).mean(1)),
            "body_ang_vel": sq(root["ang_vel_w"][:, :2]).sum(1),
            "angular_momentum": sq(self.angmom).sum(1),
            "is_terminated": terminated.float(),
            "joint_acc_l2": sq(self.joint_acc).sum(1),
            "joint_pos_limits": (torch.clamp(self.soft_lo - jp, min=0.0)
                                 + torch.clamp(jp - self.soft_hi, min=0.0)).sum(1),
            "action_rate_l2": sq(self.action - self.prev_action).sum(1),
            "foot_gait": stance_ok.float().mean(1) * active(T.FOOT_GAIT_CMD_THRESHOLD),
            "foot_clearance": (torch.abs(foot_z - T.SWING_HEIGHT) * foot_vxy).sum(1)
            * active(T.FOOT_CLEARANCE_CMD_THRESHOLD),
            "foot_slip": (sq(foot_vxy) * in_contact).sum(1) * active(T.FOOT_SLIP_CMD_THRESHOLD),
            "soft_landing": (torch.linalg.norm(self.toe_force, dim=-1) * first_contact).sum(1)
            * active(T.SOFT_LANDING_CMD_THRESHOLD),
            "stand_still": sq(jp - self.default_joint_pos).sum(1) * (total_cmd <= T.STAND_STILL_CMD_THRESHOLD),
            "alive": (~terminated).float(),
            "feet_air_time": air_rew * active(T.FEET_AIR_TIME_CMD_THRESHOLD),
            "feet_swing_height": swing_cost * active(T.FEET_SWING_CMD_THRESHOLD),
            "joint_vel_l2": sq(jv).sum(1),
            "joint_torques_l2": sq(act_frc).sum(1),
            "torque_excess": excess(act_frc - T.VISCOUS_DAMPING * jv, T.TORQUE_EXCESS_LIMIT),
            "speed_excess": excess(jv, T.SPEED_EXCESS_LIMIT),
            "base_height": sq(torch.clamp(T.HOME_BASE_HEIGHT - root["pos"][:, 2], min=0.0)),
            "track_lin_vel_filtered": torch.exp(
                -sq(cmd[:, :2] - self.vel_filtered).sum(1) / T.TRACK_LIN_STD ** 2),
            "base_vel_oscillation": sq(v_b[:, :2] - self.vel_filtered).sum(1),
            "foot_air_time_excess": torch.clamp(self.cur_air - T.FOOT_AIR_TIME_EXCESS_LIMIT, min=0.0).sum(1),
        }
        if "stand_feet_down" in T.REWARD_WEIGHTS:
            terms["stand_feet_down"] = (~in_contact).float().sum(1) * (total_cmd <= T.STAND_FEET_CMD_THRESHOLD)
        reward = torch.zeros(self.num_envs, device=self.device)
        for name, value in terms.items():
            v = torch.nan_to_num(value * T.REWARD_WEIGHTS[name] * T.STEP_DT, nan=0.0, posinf=0.0, neginf=0.0)
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
        # Each term's history contiguous, oldest first (mjlab's flattened CircularBuffer).
        order, off = [], 0
        for t in T.ACTOR_TERMS:
            for h in range(T.ACTOR_HISTORY):
                order.extend(h * self._actor_step_dim + off + np.arange(dims[t]))
            off += dims[t]
        self._actor_perm = torch.tensor(order, device=self.device)
        self.actor_dim = self._actor_step_dim * T.ACTOR_HISTORY
        self.critic_dim = sum(dims[t] for t in T.CRITIC_TERMS)

    def _observe(self) -> None:
        root = self.root
        jp = self.jp - self.default_joint_pos
        jv = self.jv
        g = (self.episode_length_buf * T.STEP_DT) % T.GAIT_PERIOD / T.GAIT_PERIOD
        phase = torch.stack([torch.sin(2 * math.pi * g), torch.cos(2 * math.pi * g)], dim=1)
        phase[torch.linalg.norm(self.command, dim=1) < T.PHASE_ZERO_BELOW] = 0.0
        force = self.toe_force.reshape(self.num_envs, -1)
        terms = {
            "command": self.command, "phase": phase, "joint_pos": jp, "joint_vel": jv,
            "actions": self.action,
            "base_ang_vel": root["ang_vel_b"], "projected_gravity": root["projected_gravity"],
            "base_lin_vel": root["lin_vel_b"],
            "foot_height": self.foot_pos[..., 2],
            "foot_air_time": self.cur_air,
            "foot_contact": self.in_contact.float(),
            "foot_contact_forces": torch.sign(force) * torch.log1p(torch.abs(force)),
        }
        critic = torch.cat([terms[t] for t in T.CRITIC_TERMS], dim=1)
        if not self.play:
            terms["joint_pos"] = jp + self._uniform(-T.JOINT_POS_NOISE, T.JOINT_POS_NOISE, jp.shape)
            terms["joint_vel"] = jv + self._uniform(-T.JOINT_VEL_NOISE, T.JOINT_VEL_NOISE, jv.shape)
        step = torch.cat([terms[t] for t in T.ACTOR_TERMS], dim=1)
        self._actor_hist = torch.roll(self._actor_hist, -1, dims=1)
        self._actor_hist[:, -1] = step
        fill = self._need_backfill
        self._actor_hist[fill] = step[fill, None, :]
        fill[:] = False
        actor = self._actor_hist.reshape(self.num_envs, -1)[:, self._actor_perm]
        self.obs = TensorDict({"actor": actor, "critic": critic}, batch_size=[self.num_envs])

    # ------------------------------------------------------------------ VecEnv

    def get_observations(self) -> TensorDict:
        return self.obs

    def step(self, actions: torch.Tensor) -> tuple[TensorDict, torch.Tensor, torch.Tensor, dict]:
        self.extras = {"log": {}}
        self.prev_action[:] = self.action
        self.action[:] = actions.detach().to(self.device)
        target = self.default_joint_pos + T.ACTION_SCALE * self.action - self.encoder_bias
        self._physics(target)
        self.episode_length_buf += 1
        self.common_step_counter += 1

        self._read_state()
        root = self.root
        time_out = self.episode_length_buf >= self.max_episode_length
        fell = torch.arccos(torch.clamp(-root["projected_gravity"][:, 2], -1.0, 1.0)) > T.FELL_OVER_ANGLE
        low = root["pos"][:, 2] < T.MIN_BASE_HEIGHT
        terminated = fell | low
        reward = self._rewards(terminated)

        done = terminated | time_out
        ids = torch.nonzero(done).flatten()
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
            self._read_state()
        self._update_commands()
        self._push()
        self._observe()
        self.extras["time_outs"] = time_out
        return self.obs, reward, done.long(), self.extras

    def set_command(self, vx: float, wz: float) -> None:
        """Pin every env to one command and restart them all (evaluation)."""
        self.fixed_command = torch.tensor([vx, 0.0, wz], device=self.device)
        self._reset(torch.arange(self.num_envs, device=self.device))
        self._read_state()
        self._update_commands()
        self._observe()

    def close(self) -> None:
        self._tmp.cleanup()
