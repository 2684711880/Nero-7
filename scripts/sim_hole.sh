#!/usr/bin/env bash
# NERO dual-arm hole demo; Gazebo only, separate ROS domain, never opens CAN.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$root/sim_ws/install/setup.bash"
pidfile="${XDG_RUNTIME_DIR:-/tmp}/nero-hole-sim-${UID}.pid"
if [[ "${1:-}" == start ]]; then
  feedback_file="$(mktemp /tmp/nero-hole-right-feedback-XXXXXX.json)"
  if [[ "${ROS_DOMAIN_ID:-0}" != 93 ]] && "$root/.venv-official/bin/python" "$root/scripts/sim_hole_right_feedback.py" "$feedback_file"; then
    echo '右臂初始姿态：收到 /right/feedback/joint_states；请确认该话题来自实机。'
    feedback_arg="right_feedback_file:=$feedback_file"
  else
    rm -f "$feedback_file"
    echo '右臂初始姿态：未收到实机 ROS 反馈，使用 park。'
    feedback_arg=''
  fi
fi
export ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$root/config/cyclonedds_pointing.xml"
case "${1:-}" in
  start)
    shift
    printf '%s\n' "$$" > "$pidfile"
    if [[ -n "$feedback_arg" ]]; then
      exec ros2 launch dual_nero_moveit_config hole_approach.launch.py "camera_calibration:=$root/config/handeye/right/d405.json" "$feedback_arg" "$@"
    fi
    exec ros2 launch dual_nero_moveit_config hole_approach.launch.py "camera_calibration:=$root/config/handeye/right/d405.json" "$@"
    ;;
  observe-plan)
    shift
    exec "$root/.venv-official/bin/python" "$root/scripts/sim_hole_observe.py" plan "$@"
    ;;
  observe-execute)
    shift
    exec "$root/.venv-official/bin/python" "$root/scripts/sim_hole_observe.py" execute "$@"
    ;;
  detect)
    shift
    exec "$root/.venv-official/bin/python" "$root/scripts/sim_hole_detect.py" --output "$root/sim_ws/validation/hole_approach/live" "$@"
    ;;
  camera)
    exec ros2 run image_tools showimage --ros-args -r image:=/sim/right_d405/camera/image_raw
    ;;
  plan)
    shift
    "$root/.venv-official/bin/python" "$root/scripts/sim_hole_detect.py" --once --no-gui
    exec "$root/.venv-official/bin/python" "$root/scripts/sim_hole_plan.py" "$@"
    ;;
  execute)
    shift
    exec "$root/.venv-official/bin/python" "$root/scripts/sim_hole_execute.py" --execute "$@"
    ;;
  retreat)
    shift
    exec "$root/.venv-official/bin/python" "$root/scripts/sim_hole_retreat.py" "$@"
    ;;
  stop)
    [[ -f "$pidfile" ]] || { echo '没有记录中的孔场景仿真进程。'; exit 0; }
    pid="$(cat "$pidfile")"
    [[ "$pid" =~ ^[0-9]+$ ]] || { echo '无效进程记录。' >&2; exit 1; }
    if ps -p "$pid" -o args= | grep -Fq 'dual_nero_moveit_config hole_approach.launch.py'; then
      kill -INT "$pid"
      echo '已请求关闭孔场景仿真。'
    else
      echo '记录的孔场景仿真已退出，没有操作其他进程。'
    fi
    rm -f "$pidfile"
    ;;
  *) echo '用法: bash scripts/sim_hole.sh {start|observe-plan|observe-execute|camera|detect|plan|execute|retreat|stop}' >&2; exit 2 ;;
esac
