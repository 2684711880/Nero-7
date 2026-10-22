#!/usr/bin/env bash
set -e
nero_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /home/a/ws_livox/setup.bash
source "$nero_root/sim_ws/install/setup.bash"
# Expose only the existing FAST-LIO package, keeping one Livox message provider.
nero_fast_prefix=/home/a/mid-360/ws/install/fast_lio
if [[ -f "$nero_fast_prefix/share/ament_index/resource_index/packages/fast_lio" ]]; then
  export AMENT_PREFIX_PATH="$nero_fast_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$nero_fast_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11489}"
export GAZEBO_MODEL_DATABASE_URI=''
cd "$nero_root/sim_ws"
exec /usr/bin/python3 "$nero_root/scripts/store_mapping_session.py" --project-root "$nero_root" "$@"
