#!/usr/bin/env bash
# 学習環境を用意する。
#
#   ./rl/scripts/setup.sh
#
# rl/ は単体で動くプログラムではなく、上流の PPO 学習基盤
# unitree_rl_mjlab に被せる overlay。上流は src/tasks/ 配下のパッケージを
# 自動 import するので、rl/ を src/tasks/velocity/config/arduino_quad として
# 置いてやれば ArduinoQuad-* のタスクが登録される。
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"

UPSTREAM_URL="https://github.com/unitreerobotics/unitree_rl_mjlab.git"
DEST="$ARDUINO_QUAD_UPSTREAM/src/tasks/velocity/config/arduino_quad"
NO_INSTALL="${1:-}"

# 上流の setup.py が指す版では、この overlay は動かない。2 つ理由がある。
#
#  1. 上流は mjlab==1.2.0 を指すが、robot_cfg.py の STS3215_ACTUATOR は
#     BuiltinPositionActuatorCfg の viscous_damping（サーボの逆起電力に相当する
#     粘性減衰）を使う。この引数は mjlab 1.3.0 から入ったので、1.2.0 では
#     robot_cfg の import が TypeError で落ちる。
#  2. mujoco は上限が指定されていないので、放っておくと最新版が入る。すると
#     mujoco-warp が mujoco.mjtEnableBit.mjENBL_MULTICCD を見つけられず、
#     import 時に AttributeError になる（3.5.0 + mujoco 3.12.0 で確認）。
#
# なので版を固定して入れ、上流のパッケージ自体は --no-deps で入れる
# （そうしないと setup.py の mjlab==1.2.0 が再び効く）。
# 別の組み合わせを試すときは環境変数で上書きできる。
#  3. warp-lang も同じ。mjlab は >=1.12 としか書いていないが、1.16 では
#     mujoco-warp のカーネルを解釈できず、学習開始時の JIT コンパイルが
#     `WarpCodegenKeyError: Referencing undefined symbol: xmat`
#     (mujoco_warp/_src/sensor.py の _frame_axis) で落ちる。
MJLAB_VERSION="${MJLAB_VERSION:-1.3.0}"
MUJOCO_WARP_VERSION="${MUJOCO_WARP_VERSION:-3.7.0.1}"
MUJOCO_VERSION="${MUJOCO_VERSION:-3.7.0}"
WARP_VERSION="${WARP_VERSION:-1.14.0}"

# 1. 上流を取ってくる
if [ -d "$ARDUINO_QUAD_UPSTREAM/.git" ]; then
  echo "==> 上流はすでにあります: $ARDUINO_QUAD_UPSTREAM"
else
  echo "==> 上流を clone します: $UPSTREAM_URL"
  mkdir -p "$(dirname "$ARDUINO_QUAD_UPSTREAM")"
  git clone --depth 1 "$UPSTREAM_URL" "$ARDUINO_QUAD_UPSTREAM"
fi

# 2. overlay を置く（シンボリックリンクなので、こちらを編集すれば即反映される）
if [ ! -d "$(dirname "$DEST")" ]; then
  echo "上流の構成が変わっています: $(dirname "$DEST") がありません" >&2
  echo "unitree_rl_mjlab 側で src/tasks/velocity/config/ の位置を確認してください" >&2
  exit 1
fi
rm -rf "$DEST"
ln -s "$ARDUINO_QUAD_ROOT/rl" "$DEST"
echo "==> overlay を置きました: $DEST -> $ARDUINO_QUAD_ROOT/rl"

# 3. 上流に互換パッチを当てる
#
# 上流の各ロボット定義（go2 / a2 / as2 / g1 / h1_2 / h2 / r1）は
# mjlab.utils.os.update_assets を import するが、この関数は mjlab 1.3.0 で
# 消えた。src/assets/robots/__init__.py はそれらを無条件に import するので、
# 1 つでも欠けると robots パッケージ全体が ImportError になり、そこから
# import される ArduinoQuad-* も登録されない。
#
# 消えたのはメッシュをディレクトリから読み込んで assets dict に詰めるだけの
# ヘルパなので、無ければこちらで足す。上流のファイルには「先頭に 1 行 import を
# 足す」以外の変更はしない。冪等（マーカーを見て二重に当てない）。
python3 - "$ARDUINO_QUAD_UPSTREAM" <<'PY'
import pathlib
import sys

upstream = pathlib.Path(sys.argv[1])
shim = upstream / "src" / "assets" / "_mjlab_compat.py"
robots_init = upstream / "src" / "assets" / "robots" / "__init__.py"
marker = "from .._mjlab_compat import apply_compat"

shim.write_text('''"""mjlab 1.3.0 以降で消えた上流依存の関数を補う。

上流の *_constants.py は ``from mjlab.utils.os import update_assets`` を書いて
いるが、この関数は mjlab 1.3.0 で削除された。中身は「ディレクトリ以下の
ファイルを読んで MjSpec.assets の dict に入れる」だけなので、無いときだけ
同じものを生やす。既にあるものは触らない。

このファイルは rl/scripts/setup.sh が上流のチェックアウトへ書き込む。
上流のコードそのものではない。
"""
import os


def apply_compat() -> None:
  import mjlab.utils.os as mjlab_os

  if hasattr(mjlab_os, "update_assets"):
    return

  def update_assets(assets, path, meshdir="", glob="*", recursive=False):
    del glob, recursive  # 上流の呼び出しは既定値しか使わない
    for dirpath, _dirs, files in os.walk(str(path)):
      for fname in files:
        fpath = os.path.join(dirpath, fname)
        rel = os.path.relpath(fpath, str(path))
        key = os.path.join(meshdir, rel) if meshdir else rel
        with open(fpath, "rb") as fh:
          assets[key] = fh.read()

  mjlab_os.update_assets = update_assets
''', encoding="utf-8")

text = robots_init.read_text(encoding="utf-8")
if marker in text:
  print("==> 互換パッチは当たっています")
else:
  robots_init.write_text(marker + "  # rl/scripts/setup.sh が追加\n"
                         "apply_compat()\n\n" + text, encoding="utf-8")
  print("==> 互換パッチを当てました:", robots_init)
PY

# 4. 依存を入れる
if [ "$NO_INSTALL" = "--no-install" ]; then
  echo "==> --no-install なので依存の導入は飛ばします"
else
  echo "==> 依存を入れます（GPU と数 GB の空きが要ります）"
  echo "    mjlab==$MJLAB_VERSION mujoco-warp==$MUJOCO_WARP_VERSION"
  echo "    mujoco==$MUJOCO_VERSION warp-lang==$WARP_VERSION"
  # scipy は mjlab 1.3.0 が terrains で import しているのに依存として宣言して
  # いない（宣言されるのは 1.5.0 から）。入れておかないとタスク登録の時点で
  # ModuleNotFoundError になる。
  pip install "mjlab==$MJLAB_VERSION" \
              "mujoco-warp==$MUJOCO_WARP_VERSION" \
              "mujoco==$MUJOCO_VERSION" \
              "warp-lang==$WARP_VERSION" \
              "scipy>=1.15"
  ( cd "$ARDUINO_QUAD_UPSTREAM" && pip install -e . --no-deps )
fi

cat <<MSG

できました。学習は次のように始めます。

    source rl/scripts/env.sh
    ./rl/scripts/train.sh ArduinoQuad-Walk --env.scene.num-envs=4096

タスク ID は ArduinoQuad-Flat / -Walk / -Robust / -Recovery の 4 つです。
歩かせたいなら -Walk を使ってください。
MSG
