#!/usr/bin/env bash
# Prepare the official gravity library and its isolated Python environment.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
vendor="$root/third_party/agilex-arm-gravity-compensation"
revision=8e2545c00ec9ec7590414946ab5f913274c7b643
if [[ ! -d "$vendor/.git" ]]; then
  git clone https://github.com/agilexrobotics/agilex-arm-gravity-compensation.git "$vendor"
  git -C "$vendor" checkout --detach "$revision"
elif [[ "$(git -C "$vendor" rev-parse HEAD)" != "$revision" ]]; then
  echo "重力补偿源码版本不同，请检查：$vendor" >&2
  exit 1
fi
git -C "$root" submodule update --init third_party/pyAgxArm
if [[ ! -x "$root/.venv-gravity/bin/python" ]]; then
  /usr/bin/python3 -m venv "$root/.venv-gravity"
fi
"$root/.venv-gravity/bin/python" -m pip install \
  'numpy==2.2.6' 'scipy==1.15.3' 'python-can==4.6.1' 'pin==4.1.0' \
  -e "$root/third_party/pyAgxArm"
echo '环境已准备；该脚本未连接或控制机械臂。'
