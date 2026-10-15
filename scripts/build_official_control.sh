#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
git submodule update --init third_party/agx_arm_ros third_party/pyAgxArm
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
colcon build --packages-up-to nero_official_commander_cpp agx_arm_ctrl
