#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "$project_root/sim_ws/install/setup.bash"
source "$project_root/hardware_ws/install/setup.bash"
set -u
exec ros2 run nero_official_commander_cpp commander "$@"
