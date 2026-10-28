#!/usr/bin/env bash
# 独立 SDK 失能：默认双臂，只在 park 附近允许发送。
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset PYTHONPATH PYTHONHOME
export PYTHONUNBUFFERED=1
exec "$root/.venv-official/bin/python" "$root/scripts/disable_at_park.py" "$@"
