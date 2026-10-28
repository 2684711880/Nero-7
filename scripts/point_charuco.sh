#!/usr/bin/env bash
set -eo pipefail
nero_point_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
exec "$nero_point_root/.venv-official/bin/python" "$nero_point_root/scripts/point_charuco.py" "$@"
