# rl — 歩行方策の学習コード

第13章の四脚を歩かせている方策 (`../walk/arduino_quad_policy.*`) を学習した
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

**GPU が手元に無い場合は Google Colab（無料枠の T4）で通せます。**
`colab/arduino_quad_colab.ipynb` が、環境構築から Arduino 用ヘッダの
書き出しまでのノートブックです。

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

ロボット記述一式（URDF・MJCF・メッシュ・RViz 設定・MJCF を SolidWorks
エクスポートから再現するスクリプト）は
[`../arduino_os_quad_robot/`](../arduino_os_quad_robot/) にある。
ROS 2 なら `ros2 launch arduino_os_quad_robot display.launch.py` で
RViz 表示できる。

## 方策の仕様（学習結果）

- 観測 87 次元（指令・歩容位相・関節角・関節速度・前回 action、各 3 履歴）。**IMU は使わない**
- MLP [96, 64]・ELU・8 出力。サーボ目標 = home 角 + 0.25 × action
- 制御 50 Hz、歩容周期 0.32 s
- シミュレーション性能: 前進 0.148 m/s、対角トロット 3.33 Hz

学習済み方策そのもの・実機で回すコード（PC 直結版 / Arduino 版）・校正
ウィザードは [`../walk/`](../walk/) にある。まず `../walk/HANDOFF.md` を読むこと。
