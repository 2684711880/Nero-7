#!/usr/bin/env bash
# Build the task into the exact hardware workspace used by run_task.sh.
set -e
nero_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$nero_root/hardware_ws"
exec colcon build --packages-select nero_official_commander_cpp "$@"
