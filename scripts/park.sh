#!/usr/bin/env bash
# 官方 SDK 独立回 park，默认双臂，不需要 ROS/MoveIt/RViz。
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset PYTHONPATH PYTHONHOME
export PYTHONUNBUFFERED=1
exec "$root/.venv-official/bin/python" "$root/scripts/move_to_park.py" "$@"
