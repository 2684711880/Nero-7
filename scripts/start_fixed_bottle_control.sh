#!/usr/bin/env bash
# Start official dual-arm control with left passive; user stops right GC first.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if pgrep -f '[n]ero_gc_official_configured.py --side right' >/dev/null; then
  echo '右臂重力补偿仍在运行。请扶稳右臂，在其终端 Ctrl+C 退出后再运行。' >&2
  exit 1
fi
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$root/config/cyclonedds_pointing.xml"
echo '右臂控制链：auto_enable=false；左臂只读；启动本身不执行轨迹。'
source "$root/scripts/official_control_env.sh"
exec ros2 launch "$root/scripts/fixed_bottle_control.launch.py" "$@"
