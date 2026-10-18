# rl/genesis — Genesis で学習し、MuJoCo で歩かせる（sim2sim）

`ArduinoQuad-Walk` を物理シミュレータ **Genesis**（GPU）で学習するコードです。
タスクの数値はすべて CPU 版の [`../cpu/task.py`](../cpu/task.py) を読み込み、
物理だけを Genesis に置き換えています。PPO は mjlab 版・CPU 版と同じ
rsl-rl-lib 5.0.1 なので、チェックポイントの形式も同じです。

学習した方策は MuJoCo で歩かせます（**sim2sim**）。`sim2sim.py` は 1 つの方策を
Genesis と MuJoCo（CPU 版の環境 = ブラウザ版と同じ物理）の両方で、同じ指令・
同じ測り方で歩かせます。ブラウザ版では「RL 方策（Genesis で学習）」
（`docs/assembly/quadruped/walk/index.html?mode=genesis`）がこの方策です。

```bash
cd docs/os-on-arduino/code/13_quadruped/rl/genesis
uv sync                                   # genesis-world 1.4.4 + rsl-rl-lib 5.0.1 + mujoco 3.7.0
export ARDUINO_QUAD_HOME_HEIGHT=0.11 ARDUINO_QUAD_GAIT_PERIOD=0.32
uv run train.py --num-envs 4096 --iterations 2300 --seed 1
uv run sim2sim.py --checkpoint logs/rsl_rl/arduino_quad_velocity/<run>/model_2299.pt
uv run sim2sim.py --npz ../../arduino_quad_policy.npz --commands "0.12,0"   # MuJoCo の方策を Genesis で

uv run python ../../host/export_quad_policy.py \
    logs/rsl_rl/arduino_quad_velocity/<run>/model_2299.pt 出力先 --backend genesis
uv run python ../../host/make_walk_bundle.py 出力先 \
    ../../../../docs/assembly/quadruped/walk/policy_genesis.json
```

`uv run python` の `python` は省かないでください。省くと uv がスクリプトの置き場所の
`host/` を別プロジェクトとして扱い、torch の無い環境で動かして失敗します。

## ファイル

| ファイル | 中身 |
|---|---|
| `gs_env.py` | ベクトル化した環境（rsl_rl の `VecEnv`）。1 ステップの中身は `../cpu/quad_env.py` と同じ順序で、すべて torch |
| `train.py` | 学習の入口。ログは `logs/rsl_rl/arduino_quad_velocity/<日時>_<名前>/`、`params/env.yaml` に `trainer: genesis` |
| `sim2sim.py` | `--sim genesis` / `mujoco` / `both`（既定）で、前進・旋回の速さ、ロール・ピッチ、転倒、歩容を出す |

## 実測

RTX 4090 1 枚で 4096 環境 × 2300 iteration が約 41 分（1 iteration 0.72〜0.79 秒。
2 本を別々の GPU で同時に回した値）。配布している `policy_genesis.json` は seed 1 の
iteration 2299 です。seed 2 は胴体のロール・ピッチが 4〜5°（seed 1 は 1〜2°）と
大きかったので採っていません。

`sim2sim.py`（64 台 × 20 秒、名目の機体、後半 10 秒の平均）:

| 指令 vx, wz | Genesis 前進 / 旋回 | MuJoCo 前進 / 旋回 | 転倒 |
|---|---|---|---|
| 0.06, 0 | 0.061 / −0.015 | 0.057 / −0.023 | 0 / 0 |
| 0.09, 0 | 0.091 / −0.002 | 0.087 / 0.001 | 0 / 0 |
| 0.12, 0 | 0.113 / 0.012 | 0.111 / 0.024 | 0 / 0 |
| 0, 0.3 | 0.000 / 0.130 | −0.006 / 0.153 | 0 / 0 |
| 0.06, −0.3 | 0.053 / −0.153 | 0.047 / −0.168 | 0 / 0 |

`--dr`（学習時のドメインランダマイゼーションを入れた機体）でも MuJoCo で 64 台とも
倒れません。足先の滑りは MuJoCo の方が 1.5 倍ほど大きくなります（20 → 30 mm/s 前後）。

## CPU 版（MuJoCo）との違い

- **床の摩擦**: Genesis は接触する 2 つの摩擦係数の大きい方を使います
  （genesis-world 1.4.4 の `genesis/engine/solvers/rigid/collider/contact.py`）。
  MuJoCo は足先の `priority` で足先の値を使うので、同じにするため床の摩擦を
  Genesis の下限の 0.01 にしています（`gs_env.py` の `FLOOR_FRICTION`）。
- **再現していない mjlab の細部**: リセット直後に 0 rad の目標が遅延線に入ること、
  報酬と終了判定が 1 物理ステップ古い状態を見ること。どちらもロボットではなく
  mjlab の実装の性質で、方策を MuJoCo（ブラウザ版）で動かすときには起きないので
  入れていません。
- **角運動量**（`angular_momentum`、重み −0.005）は MuJoCo の `subtreeangmom` の
  センサが無いので、リンクごとの質量・重心・慣性から計算しています。
- **使えないもの**: `ARDUINO_QUAD_TERRAIN=rough`（凹凸地面）と `ARDUINO_QUAD_ERFI=1`。
  指定するとエラーで止まります。
