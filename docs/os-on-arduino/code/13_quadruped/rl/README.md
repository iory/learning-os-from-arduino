# rl — 歩行方策の学習コード

第13章の四脚を歩かせている方策 (`../arduino_quad_policy.*`) を学習した
コード。紙面の都合で本には載せられなかったぶんのサポート。

## 動かす

これ自体は単体で動くプログラムではなく、
[unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab)
(mjlab + rsl_rl の PPO 学習基盤) に被せる **overlay** です。
用意から学習まで、次の 3 つで足ります。

```bash
./rl/scripts/setup.sh                    # 上流を .upstream/ に clone し、overlay を置く
source rl/scripts/env.sh                 # 機体の MJCF の場所を教える
./rl/scripts/train.sh ArduinoQuad-Walk --env.scene.num-envs=4096
```

`setup.sh` は上流を `.upstream/unitree_rl_mjlab` に clone し、この `rl/` を
`src/tasks/velocity/config/arduino_quad` として**シンボリックリンク**で
置きます。上流は `src/tasks/` 配下を自動 import するので、これだけで
`ArduinoQuad-*` の 4 タスクが登録されます。リンクなので、`rl/` を直接
編集すればそのまま反映されます。

学習した方策を再生するときは `./rl/scripts/play.sh ArduinoQuad-Walk` です。
画面の無い環境（サーバや Colab）では、ビューアを開かずに mp4 を書く
`./rl/scripts/record_video.py` を使ってください。

**GPU が手元に無い場合は、[`cpu/`](cpu/) の CPU 版で学習できます。**
同じタスクを素の MuJoCo で回すもので、6 コアのノート PC で mjlab 版を
RTX 4090 で回すのと同じくらいの速さが出ます。チェックポイントの形式も同じで、
同じエクスポータでヘッダに書き出せます。

```bash
cd rl/cpu && uv sync && uv run train.py   # 詳しくは cpu/README.md
```

Google Colab（無料枠の T4）でも通せます。`colab/arduino_quad_colab.ipynb` が、
環境構築から Arduino 用ヘッダの書き出しまでのノートブックです。

| タスク ID | 用途 |
|---|---|
| `ArduinoQuad-Flat` | 平地・素の velocity レシピ（立ち上げ確認用） |
| `ArduinoQuad-Walk` | 平地 + 歩容シェーピング（**本命。トロット歩行はこれ**） |
| `ArduinoQuad-Robust` | Walk と同じだが初期状態を崩す（実機前の頑健化） |
| `ArduinoQuad-Recovery` | 転倒姿勢から立ち上がる |

> 依存の導入には GPU と数 GB の空きが要ります。置き場所だけ先に作りたいときは
> `./rl/scripts/setup.sh --no-install` で、clone と配置だけ行えます。

### 版の固定と、上流に当てているパッチ

`setup.sh` は次の版を固定して入れます（`MJLAB_VERSION` などの環境変数で
上書きできます）。上流の `setup.py` が指すままでは動きません。

| | | なぜ |
|---|---|---|
| mjlab | 1.3.0 | `robot_cfg.py` が使う `viscous_damping`（サーボの逆起電力）と `delay_min/max_lag`（27 ms の指令遅れ）が 1.2.0 に無い。後者は sim2real の要なので、外して 1.2.0 に合わせることはできない |
| mujoco-warp | 3.7.0.1 | mjlab 1.3.0 が要求する版 |
| mujoco | 3.7.0 | 上限が指定されていないため、放っておくと最新版が入り mujoco-warp の import が `mjENBL_MULTICCD` で落ちる |
| warp-lang | 1.14.0 | 1.16 では mujoco-warp のカーネルを解釈できず、学習開始時の生成が `WarpCodegenKeyError: Referencing undefined symbol: xmat` で落ちる |
| scipy | 1.15 以上 | mjlab 1.3.0 が `terrains` で import しているのに依存として宣言していない（宣言は 1.5.0 から） |

この組み合わせは 2026-08-31 に、環境構築 → タスク登録 → 学習 30 iteration →
動画 → `.h` 書き出しまで通して確認しました（2048 環境）。

| | 1 iteration | シミュレーション速度 |
|---|---|---|
| RTX 4090（Python 3.12） | 0.77 秒 | 63,700 steps/s |
| Colab 無料枠 T4（Python 3.13） | 1.42 秒 | 34,900 steps/s |

T4 でも書籍と同じ 4,500 iteration が約 1.8 時間です。

さらに `setup.sh` は上流のチェックアウトに**互換パッチを 1 つ**当てます。
上流の各ロボット定義は mjlab 1.3.0 で消えた `mjlab.utils.os.update_assets`
を import しており、そのままだと `src/assets/robots` 全体が ImportError に
なって `ArduinoQuad-*` も登録されないためです。消えたのはメッシュを読み込む
だけのヘルパなので、無いときだけ `src/assets/_mjlab_compat.py` として足します。

## KRS 版（KrsQuad-*）

KRS 版（KRS-3302 ICS ×8、RCB-4mini）は、Feetech 版とは別の機体設定・タスク・
experiment（`krs_quad_velocity`）で学習します。Feetech 版の方策やチェックポイントは使いません。

```bash
# 1. 機体モデルを作る（CAD の URDF -> 9 リンクの URDF -> MJCF）
uv run --no-project --with numpy python krs_quad_robot/scripts/normalize_urdf.py
uv run --no-project --with 'scikit-robot>=0.3.21' --with mujoco \
    python krs_quad_robot/scripts/build_mjcf.py

# 2. 学習する（RTX 4090、4096 環境で 1 iteration 約 1.9 秒、3000 iteration で約 95 分）
./rl/scripts/train.sh KrsQuad-Walk --env.scene.num-envs=4096 --agent.max-iterations 3000

# 3. 測る（速度の追従・接地のパターン・胴体の傾き・関節とトルクの余裕。--push-scale 3 で押す）
source rl/scripts/env.sh && cd "$ARDUINO_QUAD_UPSTREAM"
python "$ARDUINO_QUAD_ROOT/rl/scripts/krs_eval.py" --ckpt <model_2999.pt>

# 4. 書き出す（include/krs_quad_policy.h、ブラウザ版の policy_krs_imu.json）
python "$ARDUINO_QUAD_ROOT/host/export_quad_policy.py" <model_2999.pt> <outdir> KrsQuad-Walk \
    --name krs_quad_policy
python "$ARDUINO_QUAD_ROOT/host/make_walk_bundle.py" <outdir> \
    "$ARDUINO_QUAD_ROOT/../../docs/assembly/quadruped/walk/policy_krs_imu.json" --name krs_quad_policy
python "$ARDUINO_QUAD_ROOT/rl/scripts/krs_web_model.py" \
    "$ARDUINO_QUAD_ROOT/../../docs/assembly/quadruped/walk/model"
```

| ファイル | 中身 |
|---|---|
| `krs_robot_cfg.py` | KRS-3302 のアクチュエータ（0.657 N·m、6.545 rad/s、±135°）、当たり判定（足先・胴体・膝・前後の脚） |
| `krs_env_cfgs.py` | 観測（IMU 込み 105 次元）、指令（前進 −0.06〜0.12 m/s、旋回 ±0.5 rad/s）、ランダム化、報酬 |
| `krs_rewards.py` | 跳ね・すべり・対角のずれ・胴体や膝の接地の罰則 |
| `krs_rl_cfg.py` | PPO の設定（ネットワークは Feetech 版と同じ [96, 64]） |
| `scripts/krs_eval.py` | 方策を数値で測る |
| `scripts/krs_web_model.py` | ブラウザ版の MJCF を mjlab の entity から書き出す |

配布している `include/krs_quad_policy.h` は `KrsQuad-Walk` の seed 3、iteration 2999 です。
3 本学習して、seed 1 は「すぐ倒れて終わる」局所解に落ちました。seed 2 は胴体の傾きが大きく、
押されると倒れやすい結果でした。どちらも採っていません。
押しを 3 倍にした `KrsQuad-Robust` での仕上げも、seed 2・seed 3 の両方で試しました。
倒れる回数も胴体の傾きも良くならなかったので、採っていません。

## 構成

ロボット定義・報酬・環境設定・学習設定を、上流に対して差し替えます:

| ファイル | 中身 |
|---|---|
| `robot_cfg.py` | 機体定義（アクチュエータ、初期姿勢、`home` 角） |
| `env_cfgs.py` | 観測・コマンド・ドメインランダマイゼーションの設定 |
| `rewards.py` | 報酬項（速度追従、歩容、姿勢、エネルギー等） |
| `rl_cfg.py` | PPO ハイパーパラメータとネットワーク構成 [96, 64] |
| `runner.py` | 学習・再生・エクスポートの入り口 |
| `_compat.py` | mjlab のバージョン差の吸収 |
| `cpu/` | 同じタスクを mjlab なしで学習する CPU 版（`task.py` に数値を一か所にまとめてある。mjlab 版との一致は `cpu/parity_check.py` で検査） |

ロボット記述一式（URDF・MJCF・メッシュ・RViz 設定・MJCF を SolidWorks
エクスポートから再現するスクリプト）は
[`../arduino_os_quad_robot/`](../arduino_os_quad_robot/) にある。
ROS 2 なら `ros2 launch arduino_os_quad_robot display.launch.py` で
RViz 表示できる。

## 方策の仕様（学習結果）

- 観測 87 次元（指令・歩容位相・関節角・関節速度・前回 action、各 3 履歴）。**IMU は使わない**
- MLP [96, 64]・ELU・8 出力。サーボ目標 = home 角 + 0.25 × action
- 制御 50 Hz、歩容周期 0.32 s
- シミュレーション性能: 指令 0.15 m/s で前進 0.147 m/s、対角トロット 3.13 Hz

学習済み方策そのもの（`../arduino_quad_policy.*` と `../include/arduino_quad_policy.h`）・
実機で回すコード（PC 直結版 `../host/`、ファームウェア `../src/`）・校正の手順は
[`../`](../) にある。まず [`../README.md`](../README.md) を読むこと。
