#!/usr/bin/env bash
# Run on the target Ubuntu/Jazzy host after installing ros-jazzy-pcl-ros.
set -euo pipefail
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
lio_ws=${1:-"$HOME/robot/lio_ws"}
upstream_commit=a8e2d0d5090af97ead8dd4fac3d37cf3dbb33ff7
mkdir -p "$lio_ws/src"
if [[ -e "$lio_ws/src/point_lio" ]]; then
    echo "Refusing to replace existing $lio_ws/src/point_lio; inspect it first." >&2
    exit 1
fi
git clone --filter=blob:none --no-checkout https://github.com/dfloreaa/point_lio_ros2.git "$lio_ws/src/point_lio"
git -C "$lio_ws/src/point_lio" sparse-checkout set --no-cone '/*' '!/image/' '!/Log/' '!/PCD/'
git -C "$lio_ws/src/point_lio" checkout --detach "$upstream_commit"
git -C "$lio_ws/src/point_lio" apply "$repo_root/deploy/lio/patches/0001-bounded-queues-and-runtime.patch"
ln -s "$repo_root/src/mrbobus_lio" "$lio_ws/src/mrbobus_lio"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$lio_ws"
MAKEFLAGS=-j1 nice -n 10 colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
