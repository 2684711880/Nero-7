#!/usr/bin/env bash
# Prepare at 10%, start official drivers at 100%, then calibrate and teleoperate.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u  # ROS setup scripts reference optional variables.
source "$root/scripts/official_control_env.sh"
set -u
if [[ $# == 0 ]]; then
  /usr/bin/python3 "$root/scripts/joycon_ready.py"
  echo "仅模型预览。执行启动：bash $root/scripts/joycon.sh --execute"
  exit 0
fi
if [[ $# == 1 && "$1" == --stop ]]; then
  exec /usr/bin/python3 -u "$root/scripts/joycon_stop.py"
fi
if [[ "$1" == --record ]]; then
  exec /usr/bin/python3 -u "$root/scripts/joycon_ready.py" "$@"
fi
if [[ $# == 1 && "$1" == --return ]]; then
  exec /usr/bin/python3 -u "$root/scripts/joycon_ready.py" --return
fi
if [[ $# == 1 && "$1" == --mirror-left-to-right ]]; then
  exec /usr/bin/python3 -u "$root/scripts/joycon_ready.py" --mirror-left-to-right
fi
if [[ $# != 1 || "$1" != --execute ]]; then
  echo "用法：bash $root/scripts/joycon.sh [--stop|--execute|--return|--record [--arm left|right|both]|--mirror-left-to-right]" >&2
  exit 2
fi
/usr/bin/python3 -u "$root/scripts/joycon_ready.py" --execute
exec /usr/bin/python3 -u "$root/scripts/joycon_process.py"
