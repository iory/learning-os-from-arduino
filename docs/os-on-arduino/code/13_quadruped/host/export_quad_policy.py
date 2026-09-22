"""Export a trained ArduinoQuad policy to a self-contained C header + metadata.

Why this exists: the policy has to run on the robot's Arduino Uno R4 WiFi, and
*every* number that connects sim to hardware -- joint order, default pose,
action scale, observation layout, history ordering, normalizer statistics -- is
decided by mjlab at runtime, not by anything a human wrote down. Getting one of
them wrong produces a robot that twitches instead of walks, with no error
message. So this script reads them all out of a live environment rather than
restating them, writes them into the header next to the weights, and then
**verifies** the exported network by re-running it in numpy and comparing
against the PyTorch actor.

Usage, for a checkpoint from the mjlab trainer (via scripts/play.sh's
environment; needs a GPU to build the env):

    ./scripts/export_quad_policy.sh <ckpt.pt> [outdir] [task]

and for one from the plain-MuJoCo trainer (rl/cpu; no GPU, no mjlab):

    cd rl/cpu && uv run ../../host/export_quad_policy.py <ckpt.pt> [outdir] --backend cpu

Both read the same constants -- the CPU trainer's env is checked against mjlab
by rl/cpu/parity_check.py -- so both produce the same header for the same
weights.

Outputs into <outdir> (default deploy/arduino_quad/):
    arduino_quad_policy.h    weights + forward pass + layout constants
    arduino_quad_policy.json the same metadata in machine-readable form
"""
import json
import os
import re
import sys
import warnings
from dataclasses import asdict

warnings.simplefilter("ignore")

import numpy as np
import torch

import argparse


def _yaml_block(text, key):
  """Return the indented block belonging to `key` in a YAML dump.

  The run's env.yaml carries `!!python/name:` tags, so yaml.safe_load refuses it
  and unsafe_load would import whatever the tags name. Only a couple of numeric
  ranges are needed here, so slice the block by indentation instead.
  """
  lines = text.splitlines()
  for i, ln in enumerate(lines):
    if ln.strip().startswith(key + ":"):
      indent = len(ln) - len(ln.lstrip())
      out = []
      for nxt in lines[i + 1:]:
        if not nxt.strip():
          continue
        ind = len(nxt) - len(nxt.lstrip())
        # A mapping child is indented deeper; a sequence child sits at the key's
        # own indent and starts with "-". Anything else ended the block.
        if ind < indent or (ind == indent and not nxt.lstrip().startswith("-")):
          break
        out.append(nxt)
      return "\n".join(out)
  return ""


def _yaml_blocks(text, key):
  """Every block belonging to `key`, in order -- the key can repeat."""
  out, rest = [], text
  while True:
    blk = _yaml_block(rest, key)
    if not blk:
      return out
    out.append(blk)
    i = rest.index(blk)
    rest = rest[i + len(blk):]


def _training_provenance(ckpt):
  """Read what the checkpoint was actually trained under, from its own run dir.

  The task name on the command line only picks the environment used to *read
  out* the layout constants; it says nothing about how the weights were
  produced. Hardware gets one artifact and no simulator, so the artifact has to
  carry that itself -- and `robustified` is measured from the saved reset
  ranges rather than asserted by whoever ran the export.
  """
  # logs/rsl_rl/<experiment>/<run>/model_NNNN.pt -- the run dir is the parent.
  run = os.path.dirname(os.path.abspath(ckpt))
  prov = {"run_dir": os.path.basename(run)}
  agent_p = os.path.join(run, "params", "agent.yaml")
  env_p = os.path.join(run, "params", "env.yaml")
  if os.path.exists(agent_p):
    a = open(agent_p).read()
    for k in ("experiment_name", "run_name", "load_run", "load_checkpoint",
              "seed"):
      m = re.search(r"^{}:\s*(.+)$".format(k), a, re.M)
      if m:
        prov[k] = m.group(1).strip()
  if os.path.exists(env_p):
    e = open(env_p).read()
    def span(blk):
      vals = [abs(float(v)) for v in re.findall(
        r"^\s*-\s*(-?\d[\d.eE+-]*)\s*$", blk, re.M)]
      return round(max(vals), 6) if vals else 0.0

    base = _yaml_block(e, "reset_base")
    pose = _yaml_block(base, "pose_range")
    joints = _yaml_block(e, "reset_robot_joints")
    # Deliberately NOT yaw: the nominal Walk run already resets to a uniformly
    # random heading, so counting it would call every run robustified.
    spans = {
      "base_roll_pitch": max(span(_yaml_block(pose, "roll")),
                             span(_yaml_block(pose, "pitch"))),
      "base_z": span(_yaml_block(pose, "z")),
      "base_velocity": span(_yaml_block(base, "velocity_range")),
      "joint_position": span(_yaml_block(joints, "position_range")),
      "joint_velocity": span(_yaml_block(joints, "velocity_range")),
    }
    prov["reset_randomization"] = spans
    # Robustification here = the fine-tune that stopped resetting every env to
    # the identical home pose at zero velocity.
    prov["robustified"] = bool(max(spans.values()) > 0.0)
  return prov



def _assert_matches_training_run(ckpt, joint_names, default_joint_pos, period):
  """Fail if the export env disagrees with the run the weights came from.

  Stance height and gait period are chosen by environment variables
  ($ARDUINO_QUAD_HOME_HEIGHT, $ARDUINO_QUAD_GAIT_PERIOD). Exporting without the
  ones the run was trained under builds a *valid* env with the wrong home pose,
  and the offsets baked into the header are then silently wrong -- the robot
  stands at one crouch and the policy was trained at another. Nothing else
  catches this, so compare against the run's own saved env.yaml.
  """
  env_p = os.path.join(os.path.dirname(os.path.abspath(ckpt)), "params",
                       "env.yaml")
  if not os.path.exists(env_p):
    print("WARNING: {} not found; cannot cross-check the export env"
          .format(env_p))
    return
  text = open(env_p).read()
  bad = []

  # env.yaml holds several `joint_pos:` blocks -- the entity's own default
  # (a bare `.*: 0.0` wildcard) as well as the scene's real home pose. Taking
  # the first one found nothing to compare and passed everything, so merge the
  # concrete name:value entries from every block instead.
  want = {}
  for blk in _yaml_blocks(text, "joint_pos"):
    for m in re.finditer(r"^\s*([A-Za-z_][\w]*):\s*(-?[\d.eE+-]+)\s*$", blk,
                         re.M):
      want[m.group(1)] = float(m.group(2))
  covered = [n for n in joint_names if n in want]
  if not covered:
    raise SystemExit(
      "cannot cross-check the export: no joint angles for {} found in {}"
      .format(joint_names[0], env_p))
  for name, have in zip(joint_names, np.asarray(default_joint_pos).ravel()):
    if name in want and abs(want[name] - float(have)) > 1e-6:
      bad.append("{}: run {:+.4f} vs export env {:+.4f}"
                 .format(name, want[name], float(have)))

  m = re.search(r"^\s*period:\s*([\d.eE+-]+)\s*$",
                _yaml_block(text, "phase"), re.M)
  if m and abs(float(m.group(1)) - float(period)) > 1e-9:
    bad.append("gait period: run {} vs export env {}"
               .format(m.group(1), period))

  if bad:
    raise SystemExit(
      "export env does not match the training run:\n  " + "\n  ".join(bad)
      + "\nSet the same $ARDUINO_QUAD_HOME_HEIGHT / $ARDUINO_QUAD_GAIT_PERIOD"
        " the run used.")
  print("export env matches the training run (home pose + gait period)")



# mjlab observation term -> the name quad_control.h knows it by. Anything not in
# here is a term the deploy code has never heard of; the export stops rather
# than writing a header whose observation vector quietly omits it.
_TERM_MACRO = {
  "base_ang_vel": "ANG_VEL",
  "projected_gravity": "GRAVITY",
  "command": "COMMAND",
  "phase": "PHASE",
  "joint_pos": "JOINT_POS",
  "joint_vel": "JOINT_VEL",
  "actions": "ACTIONS",
}
# Terms the firmware can assemble on its own. The rest need a sensor, so their
# absence has to be a compile-time fact, not a runtime surprise.
_TERM_OPTIONAL = ("ANG_VEL", "GRAVITY")


def _term_defines(layout):
  """Emit where each observation term lives, so the C side stops guessing.

  quad_control.h used to compute the offsets itself from a fixed term order.
  Training a policy with an IMU inserts a term at offset 0 and shifts everything
  after it, which that code cannot see -- it would keep building an 87-value
  vector in the old order and feed it to a 105-input network. Writing the real
  offsets next to the weights makes the two move together.
  """
  seen = {}
  for e in layout:
    m = _TERM_MACRO.get(e["term"])
    if m is None:
      raise SystemExit(
        "observation term '{}' has no deploy-side implementation; add it to "
        "quad_control.h and to _TERM_MACRO before exporting".format(e["term"]))
    seen[m] = e
  out = ["// Observation terms this policy needs, and where each one starts.\n"
         "// quad_control.h switches on these: a policy trained with an IMU and\n"
         "// firmware built without one is a compile error, not a wrong vector.\n"]
  for m in _TERM_OPTIONAL:
    e = seen.get(m)
    out.append("#define QUAD_HAS_{} {}\n".format(m, 1 if e else 0))
    out.append("#define QUAD_OFF_{} {}\n".format(m, e["offset"] if e else 0))
  for m, e in sorted(seen.items(), key=lambda kv: kv[1]["offset"]):
    if m not in _TERM_OPTIONAL:
      out.append("#define QUAD_OFF_{} {}\n".format(m, e["offset"]))
  hist = {e["history"] for e in layout}
  if len(hist) != 1:
    raise SystemExit("terms disagree on history length: {}".format(hist))
  return "".join(out)


# Set by main() from the command line.
CKPT = OUTDIR = TASK = DEV = ""


def _read_env_mjlab():
  """Layout constants from a live mjlab env, and the runner holding the actor."""
  import src.tasks  # noqa: F401  (registers tasks)
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
  from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls

  env_cfg = load_env_cfg(TASK, play=True)
  env_cfg.scene.num_envs = 1
  agent_cfg = load_rl_cfg(TASK)

  base = ManagerBasedRlEnv(cfg=env_cfg, device=DEV)
  env = RslRlVecEnvWrapper(base, clip_actions=agent_cfg.clip_actions)
  runner_cls = load_runner_cls(TASK) or MjlabOnPolicyRunner
  runner = runner_cls(env, asdict(agent_cfg), device=DEV)
  runner.load(CKPT)

  robot = base.scene["robot"]
  action_term = base.action_manager.get_term("joint_pos")

  # --- everything the firmware has to agree with, read from the live env ---
  joint_names = list(robot.joint_names)
  # The action term drives a subset in its own order; that order is what the
  # network's 8 outputs mean.
  act_ids = action_term._target_ids
  if isinstance(act_ids, torch.Tensor):
    act_ids = act_ids.detach().cpu().numpy()
  elif isinstance(act_ids, slice):
    act_ids = np.arange(len(joint_names))[act_ids]
  act_joint_names = [joint_names[int(i)] for i in np.asarray(act_ids).reshape(-1)]
  default_joint_pos = robot.data.default_joint_pos[0].detach().cpu().numpy()
  act_scale = action_term._scale
  act_scale = (np.full(len(act_joint_names), float(act_scale))
               if np.isscalar(act_scale) or getattr(act_scale, "ndim", 1) == 0
               else act_scale[0].detach().cpu().numpy())
  act_offset = action_term._offset
  act_offset = (np.full(len(act_joint_names), float(act_offset))
                if np.isscalar(act_offset) or getattr(act_offset, "ndim", 1) == 0
                else act_offset[0].detach().cpu().numpy())

  actor_group = base.observation_manager._group_obs_term_cfgs["actor"]
  actor_terms = base.observation_manager._group_obs_term_names["actor"]
  actor_dims = base.observation_manager.group_obs_term_dim["actor"]
  layout, cursor = [], 0
  for name, cfg_t, dim in zip(actor_terms, actor_group, actor_dims):
    total = int(np.prod(dim))
    hist = int(getattr(cfg_t, "history_length", 0)) or 1
    layout.append({
      "term": name, "offset": cursor, "size": total,
      "history": hist, "size_per_step": total // hist,
      # CircularBuffer.buffer is chronological, oldest -> newest.
      "order": "oldest_to_newest",
    })
    cursor += total
  period = float(actor_group[actor_terms.index("phase")].params["period"])
  return runner, {
    "joint_names": joint_names, "act_joint_names": act_joint_names,
    "default_joint_pos": default_joint_pos, "act_scale": act_scale,
    "act_offset": act_offset, "layout": layout, "clip_actions": agent_cfg.clip_actions,
    "control_dt": float(base.step_dt), "gait_period": period,
  }


def _read_env_cpu():
  """The same constants from the plain-MuJoCo trainer (rl/cpu)."""
  sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  os.pardir, "rl", "cpu"))
  import task as T
  from quad_env import QuadEnv
  from rsl_rl.runners import OnPolicyRunner

  env = QuadEnv(1, play=True, domain_rand=False)
  agent_cfg = T.ppo_runner_cfg()
  runner = OnPolicyRunner(env, agent_cfg, None, DEV)
  runner.load(CKPT)
  n = len(T.JOINT_NAMES)
  per_step = {"command": 3, "phase": 2, "joint_pos": n, "joint_vel": n, "actions": n}
  layout, cursor = [], 0
  for name in T.ACTOR_TERMS:
    total = per_step[name] * T.ACTOR_HISTORY
    layout.append({"term": name, "offset": cursor, "size": total,
                   "history": T.ACTOR_HISTORY, "size_per_step": per_step[name],
                   "order": "oldest_to_newest"})
    cursor += total
  assert cursor == env.actor_dim, (cursor, env.actor_dim)
  default = np.asarray(T.DEFAULT_JOINT_POS, dtype=np.float64)
  return runner, {
    "joint_names": list(T.JOINT_NAMES), "act_joint_names": list(T.JOINT_NAMES),
    "default_joint_pos": default, "act_scale": np.full(n, T.ACTION_SCALE),
    "act_offset": default, "layout": layout, "clip_actions": agent_cfg["clip_actions"],
    "control_dt": T.STEP_DT, "gait_period": T.GAIT_PERIOD,
  }


def main() -> int:
  global CKPT, OUTDIR, TASK, DEV
  ap = argparse.ArgumentParser(description=__doc__,
                               formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("ckpt")
  ap.add_argument("outdir", nargs="?", default="deploy/arduino_quad")
  ap.add_argument("task", nargs="?", default="ArduinoQuad-Walk")
  ap.add_argument("--backend", choices=("mjlab", "cpu"), default="mjlab",
                  help="which trainer's env to read the layout from")
  args = ap.parse_args()
  CKPT, OUTDIR, TASK = args.ckpt, args.outdir, args.task
  DEV = "cuda:0" if args.backend == "mjlab" else "cpu"
  runner, live = (_read_env_mjlab if args.backend == "mjlab" else _read_env_cpu)()
  if args.backend == "cpu":
    TASK = TASK + " (plain MuJoCo)"
  joint_names = live["joint_names"]
  act_joint_names = live["act_joint_names"]
  default_joint_pos = live["default_joint_pos"]
  act_scale, act_offset = live["act_scale"], live["act_offset"]
  layout = live["layout"]
  cursor = sum(e["size"] for e in layout)

  sd = torch.load(CKPT, map_location="cpu", weights_only=False)["actor_state_dict"]
  mean = sd["obs_normalizer._mean"].flatten().numpy().astype(np.float32)
  # rsl_rl's EmpiricalNormalization divides by (_std + eps), not by _std. That
  # is not cosmetic here: lin_vel_y is pinned to zero for this robot, so three
  # of the 87 inputs have _std == 0 exactly and dividing by _std alone gives
  # NaN. Read eps off the live module instead of assuming the default.
  eps = None
  for mod in runner.alg.actor.modules():
    if type(mod).__name__ == "EmpiricalNormalization":
      eps = float(mod.eps)
      break
  if eps is None:
    raise SystemExit("could not find the actor's EmpiricalNormalization module; "
                     "cannot know the normalizer epsilon")
  std = sd["obs_normalizer._std"].flatten().numpy().astype(np.float32) + np.float32(eps)
  layers = []
  i = 0
  while f"mlp.{i}.weight" in sd:
    layers.append((sd[f"mlp.{i}.weight"].numpy().astype(np.float32),
                   sd[f"mlp.{i}.bias"].numpy().astype(np.float32)))
    i += 2
  obs_dim, act_dim = layers[0][0].shape[1], layers[-1][0].shape[0]
  assert cursor == obs_dim, f"layout {cursor} != actor input {obs_dim}"
  assert act_dim == len(act_joint_names)

  # --- verify: numpy re-implementation vs the PyTorch actor ----------------
  def elu(x):
    return np.where(x > 0.0, x, np.expm1(np.minimum(x, 0.0)))

  def forward_np(obs):
    x = (obs - mean) / std
    for w, b in layers[:-1]:
      x = elu(x @ w.T + b)
    return x @ layers[-1][0].T + layers[-1][1]

  policy = runner.get_inference_policy(device=DEV)
  rng = np.random.default_rng(0)
  probe = rng.normal(size=(64, obs_dim)).astype(np.float32)
  probe_t = torch.as_tensor(probe, device=DEV)
  with torch.inference_mode():
    # rsl_rl models take a dict of observation groups.
    try:
      ref = policy(probe_t).cpu().numpy()
    except (IndexError, TypeError):
      ref = policy({"actor": probe_t, "policy": probe_t}).cpu().numpy()
  err = float(np.abs(ref - forward_np(probe)).max())
  print(f"export check: max |torch - exported| = {err:.3e} over 64 random obs")
  # NOT `if err > tol`: a NaN compares False against everything and would let a
  # broken export through silently.
  if not err <= 1e-4:
    raise SystemExit(f"exported network does not reproduce the actor ({err:.3e})")

  meta = {
    # The env the layout constants below were read out of -- NOT necessarily the
    # env the weights were trained in. Robust/Walk share every constant here
    # (they differ only in reset randomization), which is why exporting a Robust
    # checkpoint through the Walk env is sound; `training` records what actually
    # produced the weights.
    "export_env_task": TASK,
    "training": _training_provenance(CKPT),
    "checkpoint": os.path.abspath(CKPT),
    "obs_dim": int(obs_dim),
    "act_dim": int(act_dim),
    "hidden_dims": [int(w.shape[0]) for w, _ in layers[:-1]],
    "activation": "elu",
    "obs_layout": layout,
    "action_joint_order": act_joint_names,
    "action_scale": act_scale.tolist(),
    "action_offset_default_joint_pos": act_offset.tolist(),
    "all_joint_order": joint_names,
    "default_joint_pos": default_joint_pos.tolist(),
    "clip_actions": live["clip_actions"],
    "control_dt": live["control_dt"],
    "gait_period_s": live["gait_period"],
    "obs_normalizer_eps": eps,
    "verify_max_abs_err": err,
  }

  _assert_matches_training_run(CKPT, act_joint_names, act_offset,
                               meta["gait_period_s"])

  os.makedirs(OUTDIR, exist_ok=True)
  with open(os.path.join(OUTDIR, "arduino_quad_policy.json"), "w") as f:
    json.dump(meta, f, indent=2)

  # Same weights again as .npz, for the host-side Python controller. A laptop
  # driving the servo bus over USB is the easiest way to bring this robot up --
  # no MCU flashing, and every intermediate value is printable -- so that path
  # gets first-class artifacts rather than being told to parse the C header.
  npz = {"obs_mean": mean, "obs_std": std,
         "default_joint_pos": act_offset.astype(np.float32),
         "action_scale": act_scale.astype(np.float32)}
  for li, (w, b) in enumerate(layers):
    npz["w%d" % li] = w
    npz["b%d" % li] = b
  npz["n_layers"] = np.int32(len(layers))
  np.savez(os.path.join(OUTDIR, "arduino_quad_policy.npz"), **npz)
  print("wrote {}".format(os.path.join(OUTDIR, "arduino_quad_policy.npz")))

  def cfloat(v):
    # "%.9g" of 0.0 is "0", and "0f" is not a valid C float literal -- force a
    # decimal point on anything that came out as a bare integer.
    t = "{:.9g}".format(float(v))
    if "." not in t and "e" not in t and "n" not in t:
      t += ".0"
    return t + "f"

  def carr(name, a):
    a = np.asarray(a, dtype=np.float32).ravel()
    body = ",".join(cfloat(v) for v in a)
    return "static const float {}[{}] = {{{}}};\n".format(name, a.size, body)

  h = [
    "// GENERATED by scripts/export_quad_policy.py -- do not edit.\n",
    "// export env={}  ckpt={}\n".format(TASK, os.path.basename(CKPT)),
    "// trained in run={}  robustified={}\n".format(
      meta["training"].get("run_dir"), meta["training"].get("robustified")),
    "// verified against the PyTorch actor: max abs diff {:.3e}\n".format(err),
    "//\n// Observation layout (float, in this order; each term keeps its own\n"
    "// history, oldest step first):\n",
  ]
  for e in layout:
    h.append("//   [{:3d}..{:3d}) {:12s} {} x {}\n".format(
      e["offset"], e["offset"] + e["size"], e["term"], e["history"],
      e["size_per_step"]))
  h += [
    "//\n// Joint order for the {} outputs (and for joint_pos/joint_vel):\n"
    "//   {}\n".format(act_dim, ", ".join(act_joint_names)),
    "// Servo target = default_joint_pos + action_scale * action.\n",
    "// Control period {:.4f} s, gait period {:.3f} s.\n".format(
      meta["control_dt"], meta["gait_period_s"]),
    "#pragma once\n#include <math.h>\n\n",
    "#define QUAD_OBS_DIM {}\n#define QUAD_ACT_DIM {}\n".format(obs_dim, act_dim),
    "#define QUAD_HISTORY {}\n".format(layout[0]["history"]),
    "#define QUAD_CONTROL_DT {:.6f}f\n".format(meta["control_dt"]),
    "#define QUAD_GAIT_PERIOD {:.6f}f\n".format(meta["gait_period_s"]),
    _term_defines(layout),
    "#define QUAD_ACTION_CLIP {:.6f}f\n\n".format(
      float(meta["clip_actions"]) if meta["clip_actions"] else 0.0),
    carr("QUAD_OBS_MEAN", mean),
    # already includes the normalizer epsilon: normalised = (x - mean) / std
    carr("QUAD_OBS_STD", std),
    carr("QUAD_DEFAULT_JOINT_POS", act_offset),
    carr("QUAD_ACTION_SCALE", act_scale),
  ]
  for li, (w, b) in enumerate(layers):
    h.append("// layer {}: {} -> {}\n".format(li, w.shape[1], w.shape[0]))
    h.append(carr("QUAD_W{}".format(li), w))   # row-major, [out][in]
    h.append(carr("QUAD_B{}".format(li), b))
  dims = [layers[0][0].shape[1]] + [w.shape[0] for w, _ in layers]
  h.append("\nstatic inline float quad_elu(float x) "
           "{ return x > 0.0f ? x : expm1f(x); }\n\n")
  h.append("// obs: QUAD_OBS_DIM raw (un-normalised) values; act: QUAD_ACT_DIM out.\n")
  h.append("static inline void quad_policy_forward(const float *obs, float *act) {\n")
  h.append("  float a[{}], b[{}];\n".format(max(dims), max(dims)))
  h.append("  for (int i = 0; i < {}; ++i) a[i] = "
           "(obs[i] - QUAD_OBS_MEAN[i]) / QUAD_OBS_STD[i];\n".format(dims[0]))
  for li in range(len(layers)):
    nin, nout = dims[li], dims[li + 1]
    last = li == len(layers) - 1
    h.append("  for (int o = 0; o < {}; ++o) {{\n".format(nout))
    h.append("    float s = QUAD_B{}[o];\n".format(li))
    h.append("    const float *w = &QUAD_W{}[o * {}];\n".format(li, nin))
    h.append("    for (int i = 0; i < {}; ++i) s += w[i] * a[i];\n".format(nin))
    h.append("    b[o] = {};\n".format("s" if last else "quad_elu(s)"))
    h.append("  }\n")
    if last:
      h.append("  for (int o = 0; o < {}; ++o) act[o] = b[o];\n".format(nout))
    else:
      h.append("  for (int o = 0; o < {}; ++o) a[o] = b[o];\n".format(nout))
  h.append("}\n")

  path = os.path.join(OUTDIR, "arduino_quad_policy.h")
  with open(path, "w") as f:
    f.writelines(h)
  nparam = sum(w.size + b.size for w, b in layers)
  print("wrote {} ({} parameters, {:.1f} KB as float32)".format(
    path, nparam, nparam * 4 / 1024))
  _check_generated_c(OUTDIR, obs_dim, act_dim, forward_np)
  print("wrote {}".format(os.path.join(OUTDIR, "arduino_quad_policy.json")))
  print("action joint order:", act_joint_names)
  return 0


def _check_generated_c(outdir, obs_dim, act_dim, forward_np) -> None:
  """Compile the generated header with the host cc and compare against numpy.

  The weights are verified against PyTorch above, but the C forward pass is
  freshly generated text -- an indexing slip or an unrepresentable literal only
  shows up when a compiler actually reads it. Skipped (loudly) with no cc.
  """
  import shutil
  import subprocess
  import tempfile

  cc = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
  if cc is None:
    print("C check SKIPPED: no cc/gcc/clang on PATH")
    return

  main_c = """
#include <stdio.h>
#include "arduino_quad_policy.h"
int main(void) {
  float obs[QUAD_OBS_DIM], act[QUAD_ACT_DIM];
  int n; if (scanf("%d", &n) != 1) return 1;
  for (int k = 0; k < n; ++k) {
    for (int i = 0; i < QUAD_OBS_DIM; ++i) if (scanf("%f", &obs[i]) != 1) return 1;
    quad_policy_forward(obs, act);
    for (int o = 0; o < QUAD_ACT_DIM; ++o) printf("%.9g ", act[o]);
    printf("\\n");
  }
  return 0;
}
"""
  with tempfile.TemporaryDirectory() as td:
    src = os.path.join(td, "check.c")
    exe = os.path.join(td, "check")
    with open(src, "w") as f:
      f.write(main_c)
    subprocess.run([cc, "-std=c99", "-O2", "-Wall", "-Werror", "-I", outdir,
                    "-o", exe, src, "-lm"], check=True)
    rng = np.random.default_rng(1)
    probe = rng.normal(size=(32, obs_dim)).astype(np.float32)
    stdin = "32\n" + "\n".join(" ".join("%.9g" % v for v in r) for r in probe)
    out = subprocess.run([exe], input=stdin, capture_output=True, text=True,
                         check=True).stdout
    got = np.array([[float(x) for x in line.split()]
                    for line in out.strip().splitlines()], dtype=np.float32)
    err = float(np.abs(got - forward_np(probe)).max())
    print("C check: compiled clean; max |C - numpy| = {:.3e}".format(err))
    if not err <= 1e-4:
      raise SystemExit("generated C does not match the exported weights")


if __name__ == "__main__":
  sys.exit(main())
