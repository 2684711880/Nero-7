#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/official_control_env.sh"
case "${1:-}" in
  capture) shift; exec "$root/.venv-official/bin/python" "$root/scripts/real_hole_capture.py" "$@" ;;
  observe) shift; exec "$root/.venv-official/bin/python" "$root/scripts/real_hole_observe.py" "$@" ;;
  targets) shift; exec "$root/.venv-official/bin/python" "$root/scripts/real_hole_targets.py" "$@" ;;
  two-stage) shift; exec "$root/.venv-official/bin/python" "$root/scripts/real_hole_two_stage.py" "$@" ;;
  board) shift; exec "$root/.venv-official/bin/python" "$root/scripts/real_hole_board_scene.py" "$@" ;;
  auto-observe) shift; exec "$root/.venv-official/bin/python" "$root/scripts/real_hole_auto_observe.py" "$@" ;;
  wrist) shift; exec "$root/.venv-official/bin/python" "$root/scripts/real_hole_observe.py" --wrist-joint 7 "$@" ;;
  reset-preview) shift; exec "$root/.venv-official/bin/python" "$root/scripts/real_hole_preview.py" "$@" ;;
  *) echo '用法: bash scripts/real_hole.sh {capture|observe|auto-observe|wrist|targets|two-stage|board|reset-preview}' >&2; exit 2 ;;
esac
