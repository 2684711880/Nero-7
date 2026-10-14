#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS setup scripts can read unset variables; temporarily relax nounset.
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$project_root/sim_ws"
colcon build --packages-up-to mobile_dual_nero
