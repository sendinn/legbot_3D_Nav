#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ]]; then
  echo "Use a fresh terminal: this branch builds against ROS 2 Humble." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
export CC=/usr/bin/gcc CXX=/usr/bin/g++
export ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-${PWD}/third_party/onnxruntime}"
export CMAKE_PREFIX_PATH="${PWD}/third_party/libtorch:${PWD}/third_party/unitree_sdk2/install:${PWD}/third_party/Livox-SDK2/install:${PWD}/third_party/pct/install:${CMAKE_PREFIX_PATH:-}"
# Optional workspace-local ROS development dependencies, extracted from Humble debs.
legbot_dep_prefix="${LEGBOT_DEP_PREFIX:-${PWD}/.deps/humble/opt/ros/humble}"
if [[ -d "$legbot_dep_prefix" ]]; then
  export CMAKE_PREFIX_PATH="$legbot_dep_prefix:$CMAKE_PREFIX_PATH"
  export AMENT_PREFIX_PATH="$legbot_dep_prefix:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$legbot_dep_prefix/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$legbot_dep_prefix/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
  legbot_dep_usr="$(realpath "$legbot_dep_prefix/../../../usr")"
  export LIBRARY_PATH="$legbot_dep_usr/lib/$(gcc -dumpmachine):${LIBRARY_PATH:-}"
  export LD_LIBRARY_PATH="$legbot_dep_usr/lib/$(gcc -dumpmachine):$LD_LIBRARY_PATH"
fi
export MAKEFLAGS="-j${LEGBOT_BUILD_JOBS:-1}"
export CMAKE_BUILD_PARALLEL_LEVEL="${LEGBOT_BUILD_JOBS:-1}"
colcon build --base-paths src --executor sequential "$@" --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF
