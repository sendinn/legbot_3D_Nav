#!/usr/bin/env bash
# Source from a fresh terminal; keep Jazzy libraries out of the Humble process.
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ]]; then
  echo "This branch requires ROS 2 Humble; open a fresh terminal." >&2
  return 1 2>/dev/null || exit 1
fi
legbot_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$legbot_workspace/install/setup.bash" ]]; then
  echo "Build this workspace first: ./tools/build.sh" >&2
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/humble/setup.bash
source "$legbot_workspace/install/setup.bash"
if [[ -d "$legbot_workspace/.venv-humble/bin" ]]; then
  export PATH="$legbot_workspace/.venv-humble/bin:$PATH"
fi
export LEGBOT_DATA_DIR="$legbot_workspace/data"
export ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-$legbot_workspace/third_party/onnxruntime}"
for legbot_library in "$legbot_workspace"/third_party/{onnxruntime,libtorch}/lib \
  "$legbot_workspace/third_party/libtorch/torch.libs" \
  "$legbot_workspace"/third_party/{pct,Livox-SDK2,unitree_sdk2}/install/lib; do
  if [[ -d "$legbot_library" ]]; then
    export LD_LIBRARY_PATH="$legbot_library:${LD_LIBRARY_PATH:-}"
  fi
done
legbot_dep_prefix="${LEGBOT_DEP_PREFIX:-$legbot_workspace/.deps/humble/opt/ros/humble}"
if [[ -d "$legbot_dep_prefix" ]]; then
  export AMENT_PREFIX_PATH="$legbot_dep_prefix:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$legbot_dep_prefix/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$legbot_dep_prefix/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset legbot_workspace legbot_library legbot_dep_prefix
