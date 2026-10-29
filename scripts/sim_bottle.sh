#!/usr/bin/env bash
# NERO dual-arm hole demo; Gazebo only, separate ROS domain, never opens CAN.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$root/sim_ws/install/setup.bash"
pidfile="${XDG_RUNTIME_DIR:-/tmp}/nero-bottle-sim-${UID}.pid"
export ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$root/config/cyclonedds_pointing.xml"
case "${1:-}" in
  start)
    shift
    printf '%s\n' "$$" > "$pidfile"
    exec ros2 launch dual_nero_moveit_config bottle_hole.launch.py "camera_calibration:=$root/config/handeye/right/d405.json" startup_arm_pose:=hole_ready "$@"
    ;;
  observe-plan)
    echo "固定相机无需观察位，直接 detect / plan。"; exit 2;;
  detect)
    shift
    exec "$root/.venv-official/bin/python" "$root/scripts/sim_bottle_detect.py" --output "$root/sim_ws/validation/bottle_hole/live" "$@"
    ;;
  camera)
    exec ros2 run image_tools showimage --ros-args -r image:=/sim/middle/camera/image_raw
    ;;
  plan)
    shift
    if [[ -f "$root/sim_ws/validation/bottle_hole/latest/plan.json" ]]; then
      mv "$root/sim_ws/validation/bottle_hole/latest/plan.json" "$root/sim_ws/validation/bottle_hole/latest/previous_plan.json"
    fi
    "$root/.venv-official/bin/python" "$root/scripts/sim_bottle_detect.py" --once --no-gui
    exec "$root/.venv-official/bin/python" "$root/scripts/sim_bottle_plan.py" "$@"
    ;;
  execute)
    shift
    exec "$root/.venv-official/bin/python" "$root/scripts/sim_bottle_execute.py" --execute "$@"
    ;;
  retreat)
    echo "瓶子场景暂未实现回退。"; exit 2;;
  stop)
    [[ -f "$pidfile" ]] || { echo '没有记录中的孔场景仿真进程。'; exit 0; }
    pid="$(cat "$pidfile")"
    [[ "$pid" =~ ^[0-9]+$ ]] || { echo '无效进程记录。' >&2; exit 1; }
    if ps -p "$pid" -o args= | grep -Fq 'dual_nero_moveit_config bottle_hole.launch.py'; then
      kill -INT "$pid"
      echo '已请求关闭孔场景仿真。'
    else
      echo '记录的孔场景仿真已退出，没有操作其他进程。'
    fi
    rm -f "$pidfile"
    ;;
  *) echo '用法: bash scripts/sim_bottle.sh {start|camera|detect|plan|execute|retreat|stop}' >&2; exit 2 ;;
esac
