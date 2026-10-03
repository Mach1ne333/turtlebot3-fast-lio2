#!/usr/bin/env bash
# Download pinned upstream sources and apply only this project's FAST_LIO delta.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
git submodule update --init --recursive
LIO="$ROOT/fast_lio2_ws/src/FAST_LIO"
PIN=a4743b095409588842a5b30ddfa27e29d2f99164
[ "$(git -C "$LIO" rev-parse HEAD)" = "$PIN" ] || {
  echo "FAST_LIO submodule is not at tested commit $PIN" >&2; exit 1;
}
patch="$ROOT/patches/fast_lio2_gazebo.patch"
if git -C "$LIO" apply --reverse --check "$patch" 2>/dev/null; then
  echo 'FAST_LIO patch already applied'
else
  git -C "$LIO" apply --check "$patch"
  git -C "$LIO" apply "$patch"
fi
for kind in config rviz; do
  cp "$ROOT/patches/fast_lio2_assets/$kind/"* "$LIO/$kind/"
done

# These official packages provide the original 2D Burger, world and meshes.
# Do not vendor them in this repository; pin the known working Humble commits.
mkdir -p "$ROOT/turtlebot3_ws/src"
for spec in \
  'turtlebot3 90a68bd2e3c61c12966779da89d8eeaec82730e9' \
  'turtlebot3_simulations a35a56c8b04877dc89772b598084d8ce648a9023' \
  'turtlebot3_msgs cdee60f7618b9ed2f85f65259c0f96e3f02b1c55'; do
  read -r name commit <<< "$spec"
  dst="$ROOT/turtlebot3_ws/src/$name"
  if [ ! -e "$dst" ]; then
    git clone --branch humble --depth 1 "https://github.com/ROBOTIS-GIT/$name.git" "$dst"
  fi
  if [ "$(git -C "$dst" rev-parse HEAD)" != "$commit" ]; then
    git -C "$dst" fetch --depth 1 origin "$commit"
    git -C "$dst" checkout --detach "$commit"
  fi
done
printf '\nSources ready. Next: see README build instructions.\n'
printf 'FAST_LIO submodule is expected to appear modified locally after applying the patch.\n'
