#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
git submodule update --init third_party/agx_arm_ros third_party/pyAgxArm
git -C third_party/agx_arm_ros submodule update --init src/agx_arm_description/agx_arm_urdf
vendor_patch="$project_root/deps/agx_arm_ros-local.patch"
if git -C third_party/agx_arm_ros apply --reverse --check "$vendor_patch" >/dev/null 2>&1; then
  echo 'Official ROS local patch already applied.'
elif git -C third_party/agx_arm_ros apply --check "$vendor_patch"; then
  git -C third_party/agx_arm_ros apply "$vendor_patch"
else
  echo 'Official ROS patch conflicts with local changes; no files were overwritten.' >&2
  exit 1
fi
if [[ ! -d third_party/open_nero/.git ]]; then
  git clone https://github.com/agilexrobotics/open_nero.git third_party/open_nero
  git -C third_party/open_nero checkout --detach 9164ce5b4ced1f0926a18849fc6cf70b392a4e54
fi
for package in agx_arm_description agx_arm_moveit; do
  if [[ ! -e "hardware_ws/src/$package" ]]; then
    ln -s "../../third_party/agx_arm_ros/src/$package" "hardware_ws/src/$package"
  fi
done
if [[ ! -x .venv-official/bin/python ]]; then
  /usr/bin/python3 -m venv --system-site-packages .venv-official
fi
.venv-official/bin/python -m pip install -r deps/official_control_requirements.txt -e third_party/pyAgxArm
set +u
source /opt/ros/humble/setup.bash
set -u
cd hardware_ws
# Keep ROS Python entry points on the system interpreter; the activation helper
# adds the isolated SDK environment to PYTHONPATH for runtime imports.
colcon build --packages-up-to nero_official_commander_cpp agx_arm_ctrl agx_arm_moveit nero_dual_bringup
