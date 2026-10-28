#!/usr/bin/env bash
set -eo pipefail
nero_point_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$nero_point_root/scripts/official_control_env.sh"
exec "$nero_point_root/.venv-official/bin/python" "$nero_point_root/scripts/start_pointing.py" "$@"
