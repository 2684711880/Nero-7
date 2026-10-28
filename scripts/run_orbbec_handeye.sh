#!/usr/bin/env bash
# Fixed Gemini 336L + left/can1 held 60/6 mm board. No motor commands here.
set -eo pipefail
nero_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
nero_python="${ORBBEC_HANDEYE_PYTHON:-/home/a/Documents/Codex/2026-10-23/x-z/work/venv/bin/python}"
[[ -x "$nero_python" ]] || { echo "Missing Orbbec Python: $nero_python" >&2; exit 1; }
if ! lsusb -d 2bc5:0807 | /usr/bin/grep -q .; then
  echo '未检测到 Gemini 336L USB 设备。先连接相机，再运行本命令。' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
[[ -f "$nero_root/calibration_ws/install/setup.bash" ]] || {
  echo '缺少标定插件环境，请先运行 bash scripts/build_handeye.sh' >&2; exit 1;
}
source "$nero_root/calibration_ws/install/setup.bash"
export ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 PANTHERA_HANDEYE_TF_ONLY=1
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/nero-handeye-${UID}.lock"
flock -n 9 || { echo '另一手眼标定窗口仍在运行，请先关闭。' >&2; exit 1; }
nero_session="$nero_root/calibration_data/orbbec336l_eye_to_hand_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$nero_session"
cp "$nero_root/config/handeye_orbbec336l_eye_to_hand.rviz" "$nero_session/target_config.rviz"
echo "Eye-to-Hand：固定336L，left/can1夹板；3×4，黑框60mm，间距6mm。"
echo "Session: $nero_session"
nero_inputs_pid=""; nero_rviz_pid=""
cleanup() {
  trap - EXIT INT TERM
  [[ -z "$nero_rviz_pid" ]] || kill -TERM "$nero_rviz_pid" 2>/dev/null || true
  [[ -z "$nero_inputs_pid" ]] || kill -INT "$nero_inputs_pid" 2>/dev/null || true
  for _ in {1..30}; do
    if { [[ -z "$nero_rviz_pid" ]] || ! kill -0 "$nero_rviz_pid" 2>/dev/null; } &&
       { [[ -z "$nero_inputs_pid" ]] || ! kill -0 "$nero_inputs_pid" 2>/dev/null; }; then break; fi
    sleep .1
  done
  [[ -z "$nero_rviz_pid" ]] || kill -KILL "$nero_rviz_pid" 2>/dev/null || true
  [[ -z "$nero_inputs_pid" ]] || kill -TERM "$nero_inputs_pid" 2>/dev/null || true
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
unset GTK_PATH GTK_EXE_PREFIX GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR \
  GIO_MODULE_DIR LOCPATH GSETTINGS_SCHEMA_DIR GTK_IM_MODULE_FILE SNAP_LIBRARY_PATH
"$nero_python" "$nero_root/scripts/handeye_orbbec_inputs.py" \
  --session "$nero_session" "$@" 9>&- > >(exec 9>&-; tee "$nero_session/inputs.log") 2>&1 &
nero_inputs_pid=$!
for _ in {1..150}; do
  kill -0 "$nero_inputs_pid" 2>/dev/null || { echo "输入失败：$nero_session/inputs.log" >&2; exit 1; }
  [[ ! -f "$nero_session/ready.json" ]] || break
  sleep .1
done
[[ -f "$nero_session/ready.json" ]] || { echo "未收到相机图像：$nero_session/inputs.log" >&2; exit 1; }
rviz2 -d "$nero_session/target_config.rviz" --ros-args \
  --log-level moveit_handeye_calibration_target:=warn 9>&- \
  > >(exec 9>&-; tee "$nero_session/rviz.log") 2>&1 &
nero_rviz_pid=$!
wait -n "$nero_inputs_pid" "$nero_rviz_pid"
