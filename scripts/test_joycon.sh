#!/usr/bin/env bash
# Offline regression only: dummy processes and isolated fake ROS drivers, no CAN.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
source scripts/official_control_env.sh
for test in imu gripper ready process stop; do
  /usr/bin/python3 -m unittest discover -s scripts/tests -p "test_joycon_${test}.py"
done
/usr/bin/python3 scripts/tests/test_joycon_pose_bridge.py
ROS_DOMAIN_ID=103 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp MOCK_GRIPPER_BINARY=1 \
  /usr/bin/python3 scripts/tests/test_joycon_dual_mock.py
ROS_DOMAIN_ID=103 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp MOCK_BLOCK=1 \
  /usr/bin/python3 scripts/tests/test_joycon_dual_mock.py
ROS_DOMAIN_ID=104 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp MOCK_DISCONNECT=1 \
  /usr/bin/python3 scripts/tests/test_joycon_dual_mock.py
