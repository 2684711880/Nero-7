#!/usr/bin/env bash
set -e
nero_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$nero_root/sim_ws/install/setup.bash"
# Keep this simulation separate from the real-arm ROS domain and other worlds.
export ROS_DOMAIN_ID="${NERO_CABINET_DOMAIN_ID:-76}"
export GAZEBO_MASTER_URI="${NERO_CABINET_MASTER_URI:-http://127.0.0.1:11476}"
exec ros2 launch dual_nero_moveit_config cabinet.launch.py "$@"
