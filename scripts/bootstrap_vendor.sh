#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
git submodule update --init --recursive
vendor="$project_root/sim_ws/src/agx_arm_sim"
patch_file="$project_root/deps/agx_arm_sim-cmake.patch"
if git -C "$vendor" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
  echo 'Vendor build patch already applied.'
elif git -C "$vendor" apply --check "$patch_file"; then
  git -C "$vendor" apply "$patch_file"
  echo 'Vendor build patch applied.'
else
  echo 'Vendor patch conflicts with local changes; no files were overwritten.' >&2
  exit 1
fi
