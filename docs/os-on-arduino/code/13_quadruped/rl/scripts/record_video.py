"""方策を再生して mp4 に落とす(ビューアを開かない)。

上流の `play.py` は必ずビューア(native / viser)を開いて、その場でブロックする。
Colab や表示の無いサーバではそれが使えないので、オフスクリーンで N ステップ
回して動画だけを書くのがこのスクリプト。

2 通りの入力を取る:

  # 学習したチェックポイント(自分で学習したもの)
  python rl/scripts/record_video.py --ckpt logs/rsl_rl/arduino_quad_velocity/<run>/model_1500.pt

  # 配布済みの学習済み方策(walk/ の npz + json。実機に載っているものと同じ)
  python rl/scripts/record_video.py --bundle ../walk

`--bundle` は PyTorch ではなく walk/host/quad_policy.py の numpy 実装で回す。
実機(PC 直結版・Arduino 版)と同じコードなので、ここで歩いていれば
「エクスポートまで含めて壊れていない」ことの確認になる。

上流のチェックアウト内から実行すること(`src.tasks` を import するため):

    source rl/scripts/env.sh
    cd "$ARDUINO_QUAD_UPSTREAM"
    python "$ARDUINO_QUAD_ROOT/rl/scripts/record_video.py" --bundle "$ARDUINO_QUAD_ROOT/walk"
"""
import argparse
import json
import os
import sys
from pathlib import Path


def _parse_args() -> argparse.Namespace:
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawDescriptionHelpFormatter)
  src = p.add_mutually_exclusive_group(required=True)
  src.add_argument("--ckpt", help="学習したチェックポイント (model_*.pt)")
  src.add_argument("--bundle",
                   help="エクスポート済み方策の置き場 "
                        "(arduino_quad_policy.npz + .json のあるディレクトリ)")
  p.add_argument("--task", default=None,
                 help="タスク ID。既定は --bundle なら json の export_env_task、"
                      "--ckpt なら ArduinoQuad-Walk")
  p.add_argument("--out", default="walk.mp4", help="書き出す mp4 (既定: walk.mp4)")
  p.add_argument("--vx", type=float, default=0.09, help="前進指令 [m/s] (既定: 0.09)")
  p.add_argument("--wz", type=float, default=0.0, help="旋回指令 [rad/s] (既定: 0)")
  p.add_argument("--steps", type=int, default=300,
                 help="制御ステップ数。50 Hz なので 300 で 6 秒 (既定: 300)")
  p.add_argument("--device", default=None, help="既定: GPU があれば cuda:0")
  p.add_argument("--width", type=int, default=640)
  p.add_argument("--height", type=int, default=480)
  return p.parse_args()


def _load_bundle_meta(bundle: Path) -> dict:
  meta_path = bundle / "arduino_quad_policy.json"
  if not meta_path.exists():
    raise SystemExit(f"{meta_path} がありません(--bundle はエクスポート先を指す)")
  with open(meta_path) as f:
    return json.load(f)


def main() -> int:
  args = _parse_args()

  # 表示が無い環境でも描けるようにする。mjlab の train.py と同じ設定。
  os.environ.setdefault("MUJOCO_GL", "egl")

  # ---------------------------------------------------------------------
  # 環境の形は import 時に確定する(env_cfgs.py が環境変数を読むのはそのとき)。
  # 配布済み方策は既定とは違う姿勢・歩容周期で学習されているので、その値を
  # json から環境変数へ移してから src.tasks を import する。ずれたまま回すと、
  # エラーは出ないのに別の機体を動かすことになる。
  # ---------------------------------------------------------------------
  meta = None
  if args.bundle:
    bundle = Path(args.bundle).expanduser().resolve()
    meta = _load_bundle_meta(bundle)
    os.environ["ARDUINO_QUAD_GAIT_PERIOD"] = repr(float(meta["gait_period_s"]))
    # 姿勢は json の home 角から逆引きする。既定(0.14)のまま組むと、
    # 方策が想定していない機体を動かすことになる。
    height = _home_height_for(float(meta["default_joint_pos"][0]))
    if height and "ARDUINO_QUAD_HOME_HEIGHT" not in os.environ:
      os.environ["ARDUINO_QUAD_HOME_HEIGHT"] = height
      print(f"[record_video] 方策の home 角 {meta['default_joint_pos'][0]:.4f} rad に合わせて "
            f"ARDUINO_QUAD_HOME_HEIGHT={height} で環境を作ります")

  task = args.task or (meta["export_env_task"] if meta else "ArduinoQuad-Walk")

  from dataclasses import asdict

  import numpy as np
  import torch

  import src.tasks  # noqa: F401  (タスクを登録する)
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
  from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
  from mjlab.utils.wrappers import VideoRecorder

  device = args.device or ("cuda:0" if torch.cuda.is_available() else "cpu")

  env_cfg = load_env_cfg(task, play=True)
  env_cfg.scene.num_envs = 1
  env_cfg.viewer.width = args.width
  env_cfg.viewer.height = args.height
  # 指令を固定する。既定では毎エピソード乱数で振られるので、動画の中で
  # 勝手に向きが変わる。
  twist = env_cfg.commands["twist"]
  twist.ranges.lin_vel_x = (args.vx, args.vx)
  twist.ranges.lin_vel_y = (0.0, 0.0)   # 外転関節が無いので横移動はしない
  twist.ranges.ang_vel_z = (args.wz, args.wz)
  twist.ranges.heading = None
  twist.heading_command = False
  twist.rel_standing_envs = 0.0

  agent_cfg = load_rl_cfg(task)

  out = Path(args.out).expanduser().resolve()
  out.parent.mkdir(parents=True, exist_ok=True)

  base = ManagerBasedRlEnv(cfg=env_cfg, device=device, render_mode="rgb_array")
  env = VideoRecorder(base, video_folder=out.parent, step_trigger=lambda s: s == 0,
                      video_length=args.steps, name_prefix=out.stem,
                      disable_logger=True)

  robot = base.scene["robot"]
  action_term = base.action_manager.get_term("joint_pos")
  cmd_term = base.command_manager.get_term("twist")

  # 方策の 8 出力が「どの関節の、どの順番か」は action term が決める。
  # 名前で並べ替えず、ここから読む(export_quad_policy.py と同じ取り方)。
  target_ids = action_term._target_ids
  if isinstance(target_ids, torch.Tensor):
    target_ids = target_ids.detach().cpu().numpy()
  elif isinstance(target_ids, slice):
    target_ids = np.arange(len(robot.joint_names))[target_ids]
  act_names = [robot.joint_names[int(i)] for i in np.asarray(target_ids).reshape(-1)]
  act_idx = [robot.joint_names.index(n) for n in act_names]
  env_default_q = robot.data.default_joint_pos[0].detach().cpu().numpy()[act_idx]

  if meta is not None:
    # --- numpy 実装(実機と同じコード)で回す ---
    sys.path.insert(0, str(_find_host_dir(bundle)))
    from quad_policy import QuadPolicy

    policy = QuadPolicy(str(bundle))
    if policy.joint_names != act_names:
      raise SystemExit(
        "関節の並びが環境と方策で違います。\n"
        f"  方策: {policy.joint_names}\n"
        f"  環境: {act_names}\n"
        f"タスク ID が合っていない可能性があります(--task、いまは {task})")
    if not np.allclose(policy.default_q, env_default_q, atol=1e-3):
      hint = _home_height_for(float(policy.default_q[0]))
      raise SystemExit(
        "home 姿勢が環境と方策で違います。この方策は別の姿勢で学習されています。\n"
        f"  方策: {np.round(policy.default_q, 4).tolist()}\n"
        f"  環境: {np.round(env_default_q, 4).tolist()}\n"
        + (f"ARDUINO_QUAD_HOME_HEIGHT={hint} を付けて実行し直してください。" if hint else
           "rl/robot_cfg.py の _HOME_TABLE と見比べてください。"))

    def do_step() -> None:
      q = robot.data.joint_pos[0].detach().cpu().numpy()[act_idx]
      qd = robot.data.joint_vel[0].detach().cpu().numpy()[act_idx]
      targets, _ = policy.step(q, qd, (args.vx, 0.0, args.wz))
      # env は「home からのずれ / scale」を action として受け取る。
      raw = (targets - env_default_q) / policy.action_scale
      env.step(torch.as_tensor(raw, dtype=torch.float32,
                               device=device).unsqueeze(0))

    print(f"[record_video] {bundle}/arduino_quad_policy.npz を numpy で回します "
          f"(task={task}, 学習元={meta.get('checkpoint', '?')})")
  else:
    # --- 学習したチェックポイントを PyTorch で回す ---
    ckpt = Path(args.ckpt).expanduser().resolve()
    if not ckpt.exists():
      raise SystemExit(f"チェックポイントがありません: {ckpt}")
    wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = (load_runner_cls(task) or MjlabOnPolicyRunner)(
      wrapped, asdict(agent_cfg), device=device)
    runner.load(str(ckpt), load_cfg={"actor": True}, strict=True, map_location=device)
    inference = runner.get_inference_policy(device=device)
    obs_holder = {"obs": wrapped.get_observations()}

    def do_step() -> None:
      with torch.inference_mode():
        action = inference(obs_holder["obs"])
      obs_holder["obs"] = wrapped.step(action)[0]

    print(f"[record_video] {ckpt} を回します (task={task})")

  # ---------------------------------------------------------------------
  env.reset()
  vx_sum = 0.0
  for _ in range(args.steps):
    cmd_term.command[:, 0] = args.vx
    cmd_term.command[:, 1] = 0.0
    cmd_term.command[:, 2] = args.wz
    do_step()
    vx_sum += float(robot.data.root_link_lin_vel_b[0, 0])

  env.close()

  dt = base.step_dt
  print(f"[record_video] 指令 vx={args.vx:.3f} m/s wz={args.wz:.3f} rad/s "
        f"→ 実測 前進 {vx_sum / args.steps:.3f} m/s "
        f"({args.steps} ステップ = {args.steps * dt:.1f} s)")

  # VideoRecorder は step 番号を付けたファイル名で書く。名前を戻しておく。
  written = out.parent / f"{out.stem}-step-0.mp4"
  if written.exists():
    written.replace(out)
  if not out.exists():
    raise SystemExit(f"動画が書けませんでした: {out}")
  print(f"[record_video] 書き出しました: {out}")
  return 0


def _find_host_dir(bundle: Path) -> Path:
  """quad_policy.py(実機と同じ numpy 実装)の場所を探す。"""
  for cand in (bundle / "host", bundle.parent / "host", bundle.parent / "walk" / "host"):
    if (cand / "quad_policy.py").exists():
      return cand
  raise SystemExit(
    f"quad_policy.py が見つかりません({bundle}/host などを探しました)。"
    "--bundle には walk/ のような、host/quad_policy.py を持つ場所を渡してください")


def _home_height_for(hip_home: float) -> str:
  """方策の hip 角から、対応する胴体高さ（ARDUINO_QUAD_HOME_HEIGHT）を逆引きする。

  robot_cfg.py の _HOME_TABLE を import せずにテキストから読む。あの表は
  モジュールの import 時に環境変数を読んで home 角を確定させてしまうので、
  値を決める前に import するわけにいかない。一致しなければ空文字を返し、
  呼び出し側は環境を作ったあとの照合で落ちる。
  """
  import re
  table_src = (Path(__file__).resolve().parent.parent / "robot_cfg.py").read_text()
  best, best_err = "", 1e9
  for height, hip in re.findall(r"\n\s*(0\.\d+):\s*\(([-\d.]+),", table_src):
    err = abs(float(hip) - float(hip_home))
    if err < best_err:
      best, best_err = height, err
  return best if best_err <= 1e-3 else ""


if __name__ == "__main__":
  raise SystemExit(main())
