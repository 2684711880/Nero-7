#!/usr/bin/env bash
set -e
nero_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$nero_root/scripts/run_task.sh" "$@"
