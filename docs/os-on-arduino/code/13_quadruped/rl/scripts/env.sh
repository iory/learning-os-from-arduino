# 学習コードから機体の MJCF を見つけられるようにする。
#
#   source rl/scripts/env.sh
#
# rl/ は上流のチェックアウトへコピーされるので、相対パスでは MJCF に
# たどり着けない。そのため環境変数で場所を渡す。
_here="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

# rl/scripts -> rl -> 13_quadruped
export ARDUINO_QUAD_ROOT="$(cd "$_here/../.." && pwd)"
export ARDUINO_QUAD_XML="$ARDUINO_QUAD_ROOT/arduino_os_quad_robot/mjcf/arduino_os_quad_robot.xml"

# 上流（unitree_rl_mjlab）のチェックアウト先
export ARDUINO_QUAD_UPSTREAM="${ARDUINO_QUAD_UPSTREAM:-$ARDUINO_QUAD_ROOT/.upstream/unitree_rl_mjlab}"

if [ ! -f "$ARDUINO_QUAD_XML" ]; then
  echo "warning: MJCF が見つかりません: $ARDUINO_QUAD_XML" >&2
fi
