#!/usr/bin/env bash
# 默认双臂：left=can1、right=can0；Open Nero 侧装，标准夹爪，无相机。
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python="$root/.venv-gravity/bin/python"
if [[ ! -x "$python" ]]; then
  echo "缺少重力补偿环境: $python" >&2
  exit 1
fi
# 隔离 ROS 的 PYTHONPATH，防止 Pinocchio/NumPy 库版本混用。
unset PYTHONPATH PYTHONHOME
export PYTHONUNBUFFERED=1
exec "$python" "$root/scripts/run_nero_gravity.py" "$@"
