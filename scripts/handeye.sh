#!/usr/bin/env bash
set -euo pipefail
nero_handeye_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$nero_handeye_root/.venv-official/bin/python" "$nero_handeye_root/scripts/handeye_charuco.py" "$@"
