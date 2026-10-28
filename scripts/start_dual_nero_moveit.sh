#!/usr/bin/env bash
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/official_control_env.sh"
exec ros2 launch nero_dual_bringup dual_moveit.launch.py "$@"
