#!/usr/bin/env bash
# 学習した方策を再生する。引数はそのまま上流の scripts/play.py に渡る。
#
#   ./rl/scripts/play.sh ArduinoQuad-Walk
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
[ -d "$ARDUINO_QUAD_UPSTREAM" ] || { echo "先に ./rl/scripts/setup.sh を実行してください" >&2; exit 1; }
cd "$ARDUINO_QUAD_UPSTREAM"
exec python scripts/play.py "$@"
