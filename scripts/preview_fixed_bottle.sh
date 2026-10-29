#!/usr/bin/env bash
# Read-only planning preview; live CAN feedback only, no actuator control.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/official_control_env.sh"
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$root/config/cyclonedds_pointing.xml"
exec ros2 launch "$root/scripts/fixed_bottle_preview.launch.py" "$@"
