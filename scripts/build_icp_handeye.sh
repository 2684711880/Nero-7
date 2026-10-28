#!/usr/bin/env bash
# Build the pinned RegHEC fork and its missing dependencies inside the workspace.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
third="$root/third_party"
clone_at() {
  local url="$1" path="$2" commit="$3"
  if [[ ! -d "$path/.git" ]]; then
    git init "$path"
    git -C "$path" remote add origin "$url"
    git -C "$path" fetch --depth 1 origin "$commit"
    git -C "$path" checkout --detach FETCH_HEAD
  fi
  if [[ "$(git -C "$path" rev-parse HEAD)" != "$commit" ]]; then
    echo "已有依赖版本不同：$path；请保留现有工作并检查版本。" >&2
    exit 1
  fi
}
clone_at https://github.com/payam-nourizadeh/eye_hand_calib_pointcloud.git "$third/eye_hand_calib_pointcloud" 2cb4f0a98f595b5731490771fb551e13d20339d8
clone_at https://github.com/resibots/limbo.git "$third/reghec_limbo" 43c67a6999a40a3349aa6d0f152f853e028ed509
clone_at https://github.com/strasdat/Sophus.git "$third/reghec_sophus" d0b7315a0d90fc6143defa54596a3a95d9fa10ec
if git -C "$third/eye_hand_calib_pointcloud" apply --check "$root/scripts/reghec_compat.patch"; then
  git -C "$third/eye_hand_calib_pointcloud" apply "$root/scripts/reghec_compat.patch"
elif ! git -C "$third/eye_hand_calib_pointcloud" apply --reverse --check "$root/scripts/reghec_compat.patch"; then
  echo 'RegHEC 源码存在其他修改，无法自动应用兼容补丁。' >&2; exit 1
fi
mkdir -p "$third/reghec_deps/packages" "$third/reghec_deps/root"
if [[ ! -e "$third/reghec_deps/root/usr/include/nlopt.hpp" ]]; then
  (cd "$third/reghec_deps/packages" && apt-get download libnlopt-dev libnlopt-cxx-dev libnlopt0 libnlopt-cxx0)
  find "$third/reghec_deps/packages" -name '*.deb' -exec dpkg-deb -x '{}' "$third/reghec_deps/root" \;
fi
ln -sfn ../../../../eye_hand_calib_pointcloud/include/expSE3.hpp "$third/reghec_limbo/src/limbo/kernel/expSE3.hpp"
export LIMBO_DIR="$third/reghec_limbo"
export LD_LIBRARY_PATH="$third/reghec_deps/root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
cmake -S "$third/eye_hand_calib_pointcloud" -B "$third/reghec_build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$third/reghec_deps/root/usr" \
  -DCMAKE_CXX_FLAGS="-I$third/reghec_sophus"
cmake --build "$third/reghec_build" -j1
