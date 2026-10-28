#!/usr/bin/env bash
# Manual RViz calibration: camera + passive CAN/FK only.
set -eo pipefail
nero_handeye_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
nero_target=aruco_grid
nero_camera_only=false
nero_camera_args=()
while (($#)); do
  case "$1" in
    --camera-only) nero_camera_only=true; nero_camera_args+=("$1"); shift ;;
    --target)
      [[ $# -ge 2 ]] || { echo '--target requires aruco_grid, aruco or charuco' >&2; exit 2; }
      nero_target="$2"; shift 2 ;;
    *) nero_camera_args+=("$1"); shift ;;
  esac
done
case "$nero_target" in
  aruco_grid) nero_board_description='3×4 ArUco：DICT_5X5_250，ID 0～11，黑框30mm，间距3mm' ;;
  aruco) nero_board_description='单 ArUco：DICT_6X6_250，ID 23，黑框实测100mm' ;;
  charuco) nero_board_description='ChArUco：7×5，方格40mm，编码黑框30mm' ;;
  *) echo "Unsupported target: $nero_target (choose aruco_grid, aruco or charuco)" >&2; exit 2 ;;
esac
nero_rviz_config="$nero_handeye_root/config/handeye_${nero_target}.rviz"
source /opt/ros/humble/setup.bash
if [[ ! -f "$nero_handeye_root/calibration_ws/install/setup.bash" ]]; then
  echo '先运行 bash scripts/build_handeye.sh' >&2
  exit 1
fi
source "$nero_handeye_root/calibration_ws/install/setup.bash"
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
export PANTHERA_HANDEYE_TF_ONLY=1
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/nero-handeye-${UID}.lock"
if ! flock -n 9; then
  echo 'NERO hand-eye window is already running.' >&2
  exit 1
fi
nero_session="$nero_handeye_root/calibration_data/rviz_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$nero_session"
echo "RViz 采集日志与建议保存目录：$nero_session"
if [[ "$nero_camera_only" == true ]]; then
  echo "仅相机测试：$nero_board_description。不连接 CAN、不读取机械臂反馈；只观察图像与检测坐标轴。"
else
  echo "板参数：$nero_board_description。停稳后 Take Sample，完成后 Solve 并保存样本/结果。"
fi
cp "$nero_rviz_config" "$nero_session/target_config.rviz"
nero_inputs_pid=""
nero_rviz_pid=""
cleanup() {
  trap - EXIT INT TERM
  [[ -z "$nero_rviz_pid" ]] || kill -TERM "$nero_rviz_pid" 2>/dev/null || true
  [[ -z "$nero_inputs_pid" ]] || kill -INT "$nero_inputs_pid" 2>/dev/null || true
  for _ in {1..30}; do
    if { [[ -z "$nero_rviz_pid" ]] || ! kill -0 "$nero_rviz_pid" 2>/dev/null; } &&
       { [[ -z "$nero_inputs_pid" ]] || ! kill -0 "$nero_inputs_pid" 2>/dev/null; }; then
      break
    fi
    sleep .1
  done
  [[ -z "$nero_rviz_pid" ]] || kill -KILL "$nero_rviz_pid" 2>/dev/null || true
  [[ -z "$nero_inputs_pid" ]] || kill -TERM "$nero_inputs_pid" 2>/dev/null || true
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
"$nero_handeye_root/.venv-official/bin/python" "$nero_handeye_root/scripts/handeye_rviz_inputs.py" \
  --session "$nero_session" "${nero_camera_args[@]}" 9>&- > >(exec 9>&-; tee "$nero_session/inputs.log") 2>&1 &
nero_inputs_pid=$!
# Wait for camera rectification and live image publishing before loading the UI.
for _ in {1..100}; do
  if ! kill -0 "$nero_inputs_pid" 2>/dev/null; then
    echo "Camera/FK input failed; see $nero_session/inputs.log" >&2
    exit 1
  fi
  [[ ! -f "$nero_session/ready.json" ]] || break
  sleep .1
done
if [[ ! -f "$nero_session/ready.json" ]]; then
  echo "相机输入尚未就绪，请检查 $nero_session/inputs.log" >&2
  exit 1
fi
rviz2 -d "$nero_session/target_config.rviz" --ros-args \
  --log-level moveit_handeye_calibration_target:=warn 9>&- > >(exec 9>&-; tee "$nero_session/rviz.log") 2>&1 &
nero_rviz_pid=$!
wait -n "$nero_inputs_pid" "$nero_rviz_pid"
