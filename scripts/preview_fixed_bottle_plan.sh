#!/usr/bin/env bash
# Fresh fixed-camera observation -> planning-only RViz preview, never actuates.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/official_control_env.sh"
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$root/config/cyclonedds_pointing.xml"
if [[ "${1:-}" == --reuse ]]; then
  "$root/.venv-official/bin/python" "$root/scripts/fixed_bottle_board.py" --inspection-allow-stale
  "$root/.venv-official/bin/python" "$root/scripts/fixed_bottle_tool_scene.py"
  "$root/.venv-official/bin/python" "$root/scripts/fixed_bottle_cameras.py"
  exec "$root/.venv-official/bin/python" "$root/scripts/real_hole_two_stage.py" --targets "$root/records/fixed_bottle_targets.json" --preview-only --inspection-allow-stale
fi
if [[ $# -gt 0 ]]; then
  capture="$1"
else
  capture="$(find "$root/captures" -maxdepth 1 -type d -name 'orbbec_hole_20*' -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"
fi
[[ -n "$capture" && -f "$capture/metadata.json" ]] || { echo '找不到预览窗口按 s 保存的相机帧。' >&2; exit 1; }
"$root/.venv-official/bin/python" "$root/scripts/prepare_fixed_bottle_targets.py" "$capture"
"$root/.venv-official/bin/python" "$root/scripts/fixed_bottle_board.py"
"$root/.venv-official/bin/python" "$root/scripts/fixed_bottle_tool_scene.py"
"$root/.venv-official/bin/python" "$root/scripts/fixed_bottle_cameras.py"
exec "$root/.venv-official/bin/python" "$root/scripts/real_hole_two_stage.py" --targets "$root/records/fixed_bottle_targets.json" --preview-only
