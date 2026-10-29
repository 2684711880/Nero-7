#!/usr/bin/env bash
# Fresh frame -> scene -> plan -> human-reviewed approach and feed.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if pgrep -f '[n]ero_gc_official_configured.py --side right' >/dev/null; then
  echo '右臂重力补偿仍在运行。请扶稳右臂，在其终端 Ctrl+C 退出后再运行。' >&2
  exit 1
fi
source "$root/scripts/official_control_env.sh"
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$root/config/cyclonedds_pointing.xml"
if [[ $# -gt 0 ]]; then capture="$1"; else capture="$(find "$root/captures" -maxdepth 1 -type d -name 'orbbec_hole_20*' -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"; fi
[[ -n "$capture" && -f "$capture/metadata.json" ]] || { echo '先在中间相机窗口按 s 保存当前画面。' >&2; exit 1; }
"$root/.venv-official/bin/python" "$root/scripts/prepare_fixed_bottle_targets.py" "$capture"
"$root/.venv-official/bin/python" "$root/scripts/fixed_bottle_board.py"
"$root/.venv-official/bin/python" "$root/scripts/fixed_bottle_tool_scene.py"
"$root/.venv-official/bin/python" "$root/scripts/fixed_bottle_cameras.py"
exec "$root/.venv-official/bin/python" "$root/scripts/real_hole_two_stage.py" --targets "$root/records/fixed_bottle_targets.json" --allow-approximate-bottle
