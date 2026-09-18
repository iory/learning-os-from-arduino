#!/usr/bin/env bash
# 学習を始める。引数はそのまま上流の scripts/train.py に渡る。
#
#   ./rl/scripts/train.sh ArduinoQuad-Walk --env.scene.num-envs=4096
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
[ -d "$ARDUINO_QUAD_UPSTREAM" ] || { echo "先に ./rl/scripts/setup.sh を実行してください" >&2; exit 1; }
cd "$ARDUINO_QUAD_UPSTREAM"
exec python scripts/train.py "$@"
