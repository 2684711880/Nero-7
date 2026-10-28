#!/usr/bin/env bash
# Always build the saved task source in the workspace that will be executed.
set -e
nero_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
echo "编译当前任务：$nero_root/hardware_ws/src/nero_official_commander_cpp/src/robot_task.cpp"
bash "$nero_root/scripts/build_task.sh"
source "$nero_root/scripts/official_control_env.sh"
exec "$nero_root/hardware_ws/install/nero_official_commander_cpp/lib/nero_official_commander_cpp/point_to_point" "$@"
