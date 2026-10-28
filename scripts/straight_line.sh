#!/usr/bin/env bash
set -e
nero_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$nero_root/scripts/official_control_env.sh"
exec ros2 run nero_official_commander_cpp straight_line "$@"
