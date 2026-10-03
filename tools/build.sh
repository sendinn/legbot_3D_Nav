#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source tools/native_platform.sh
legbot_jobs="${LEGBOT_BUILD_JOBS:-$LEGBOT_DEFAULT_JOBS}"
[[ "$legbot_jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'LEGBOT_BUILD_JOBS 必须是正整数' >&2; exit 2; }
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ]]; then
  echo "Use a fresh terminal: this branch builds against ROS 2 Humble." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
export CC=/usr/bin/gcc CXX=/usr/bin/g++
export ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-${PWD}/third_party/onnxruntime}"
for legbot_library in "$PWD/third_party/libtorch/lib/libtorch.so" "$ONNXRUNTIME_ROOT/lib/libonnxruntime.so"; do
  if [[ -f "$legbot_library" ]]; then
    /usr/bin/python3 tools/check_native_library.py "$LEGBOT_ARCH" "$legbot_library"
  fi
done
if [[ -f third_party/libtorch/share/cmake/Torch/TorchConfig.cmake ]] &&
   grep -q '_GLIBCXX_USE_CXX11_ABI=0' third_party/libtorch/share/cmake/Torch/TorchConfig.cmake; then
  echo 'LibTorch 使用旧 C++ ABI，与 ROS 库不兼容；请使用 install.sh 安装对应架构的依赖。' >&2
  exit 1
fi
export LD_LIBRARY_PATH="$PWD/third_party/libtorch/lib:$PWD/third_party/libtorch/torch.libs:$ONNXRUNTIME_ROOT/lib:${LD_LIBRARY_PATH:-}"
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
export MAKEFLAGS="-j$legbot_jobs"
export CMAKE_BUILD_PARALLEL_LEVEL="$legbot_jobs"
colcon_path_args=()
# Humble's rosidl CMake reads generated path lists as ASCII strings.
if printf '%s' "$PWD" | LC_ALL=C grep -q '[^ -~]'; then
  custom_build_base=false
  for arg in "$@"; do
    case "$arg" in
      --build-base|--build-base=*) custom_build_base=true ;;
    esac
  done
  if ! $custom_build_base; then
    workspace_hash="$(printf '%s' "$PWD" | sha256sum)"
    workspace_hash="${workspace_hash%% *}"
    colcon_path_args=(--build-base "${XDG_CACHE_HOME:-$HOME/.cache}/legbot-colcon/${workspace_hash:0:16}")
    printf 'ROS IDL build directory: %s\n' "${colcon_path_args[1]}"
  fi
fi
colcon build --base-paths src --executor sequential "${colcon_path_args[@]}" "$@" --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF
