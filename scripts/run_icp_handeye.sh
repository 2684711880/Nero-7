#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$root/.venv-official/bin/python" "$root/scripts/icp_handeye.py" "$@"
