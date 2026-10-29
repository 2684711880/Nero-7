#!/usr/bin/env bash
set -euo pipefail
nero_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
nero_python="${ORBBEC_HOLE_PYTHON:-/home/a/Documents/Codex/2026-10-23/x-z/work/venv/bin/python}"
[[ -x "$nero_python" ]] || { echo "找不到 Orbbec Python: $nero_python；可用 ORBBEC_HOLE_PYTHON 指定。" >&2; exit 1; }
exec "$nero_python" "$nero_root/scripts/preview_orbbec_hole.py" "$@"
