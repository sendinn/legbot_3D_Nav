#!/usr/bin/env bash
# Ubuntu 22.04 x86_64 / ROS 2 Humble. Run as a normal user.
set -Eeuo pipefail
trap 'echo "Legbot 依赖安装失败（第 $LINENO 行），可修复后重新执行。" >&2' ERR
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
JOBS="${LEGBOT_BUILD_JOBS:-2}"
SKIP_APT=false
SKIP_MODELS=false
TOMOGRAPHY=false
usage() {
  cat <<'HELP'
用法：./install.sh [--jobs N] [--skip-apt] [--skip-models] [--with-tomography]
默认安装 ROS Humble、系统依赖、CPU 推理库、SDK、PCT 依赖和 Git LFS 策略。
--skip-apt         已具备系统/ROS 依赖时，跳过 apt 和 rosdep
--skip-models      跳过策略下载（已有模型或仅编译时使用）
--with-tomography  额外安装 Open3D/CuPy 并拉取 PCD；制图运行需 CUDA 12
--jobs N          原生依赖编译线程数，默认 2 或 LEGBOT_BUILD_JOBS
仅支持 Ubuntu 22.04 x86_64。不要使用 sudo 运行整个脚本。
HELP
}
while (($#)); do
  case "$1" in
    --jobs) [[ $# -ge 2 ]] || { usage; exit 2; }; JOBS="$2"; shift 2 ;;
    --skip-apt) SKIP_APT=true; shift ;;
    --skip-models) SKIP_MODELS=true; shift ;;
    --with-tomography) TOMOGRAPHY=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage; exit 2 ;;
  esac
done
[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || { echo '--jobs 必须是正整数' >&2; exit 2; }
[[ $EUID -ne 0 ]] || { echo '请使用普通用户运行，仅系统安装步骤调用 sudo。' >&2; exit 1; }
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 && "$(uname -m)" == x86_64 ]] || {
  echo '本分支安装器仅支持 Ubuntu 22.04 x86_64；ARM64 推理库尚未适配。' >&2; exit 1;
}
[[ -z "${ROS_DISTRO:-}" || "$ROS_DISTRO" == humble ]] || {
  echo '请在未加载其他 ROS 发行版的新终端运行。' >&2; exit 1;
}
mkdir -p .deps/downloads .deps/sources .deps/build third_party
CACHE="$ROOT/.deps/downloads"
STAGING="$(mktemp -d "$ROOT/.deps/install-stage.XXXXXX")"
trap 'rm -rf -- "$STAGING"' EXIT

# Download atomically and verify pinned artifacts before extraction.
download() {
  local url="$1" file="$2" digest="${3:-}"
  if [[ ! -f "$file" ]]; then
    curl --fail --location --retry 3 --connect-timeout 30 "$url" -o "$file.part"
    mv "$file.part" "$file"
  fi
  if [[ -n "$digest" ]] && ! echo "$digest  $file" | sha256sum --check --status; then
    echo "下载校验失败：$file；请移走该缓存后重试。" >&2
    exit 1
  fi
}
if ! $SKIP_APT; then
  sudo apt-get update
  sudo apt-get install --no-remove -y ca-certificates curl software-properties-common python3
  sudo add-apt-repository -y universe
  sudo apt-get update
  if ! apt-cache show ros-humble-desktop >/dev/null 2>&1; then
    # Official ROS repository bootstrap; release assets are provided by ros-infrastructure.
    curl -fL --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      -o "$STAGING/ros-apt-release.json"
    ROS_SOURCE_URL="$(python3 - "$STAGING/ros-apt-release.json" <<'PY'
import json, sys
release = json.load(open(sys.argv[1]))
assets = [a['browser_download_url'] for a in release['assets']
          if a['name'].startswith('ros2-apt-source_') and a['name'].endswith('.jammy_all.deb')]
if len(assets) != 1:
    raise SystemExit('未找到官方 Ubuntu 22.04 ROS apt 源安装包')
print(assets[0])
PY
)"
    download "$ROS_SOURCE_URL" "$STAGING/ros2-apt-source.deb"
    sudo apt-get install --no-remove -y "$STAGING/ros2-apt-source.deb"
    sudo apt-get update
  fi
  # Updating these together avoids early Jammy systemd/udev dependency conflicts.
  sudo apt-get install --no-remove -y systemd udev \
    build-essential cmake pkg-config git git-lfs curl unzip \
    python3-dev python3-venv python3-pip python3-numpy python3-scipy python3-yaml \
    python3-colcon-common-extensions python3-rosdep pybind11-dev \
    libeigen3-dev libboost-all-dev libpcl-dev libopencv-dev libyaml-cpp-dev libcap-dev libapr1-dev \
    libignition-gazebo6-dev libignition-plugin-dev \
    ros-humble-desktop ros-humble-ros-gz ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-pcl-ros ros-humble-cv-bridge ros-humble-tf2-sensor-msgs \
    ros-humble-backward-ros ros-humble-rmw-fastrtps-cpp ros-humble-gtsam \
    libaprutil1-dev ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
    ros-humble-interactive-markers
fi
[[ -f /opt/ros/humble/setup.bash ]] || { echo '缺少 ROS Humble，请取消 --skip-apt。' >&2; exit 1; }
set +u
source /opt/ros/humble/setup.bash
set -u
export LANG=C.UTF-8
if ! $SKIP_APT; then
  mkdir "$STAGING/rosdep"
  download \
    'https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list' \
    "$STAGING/rosdep/20-default.list"
  export ROSDEP_SOURCE_PATH="$STAGING/rosdep"
  export ROSDISTRO_INDEX_URL="${ROSDISTRO_INDEX_URL:-https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml}"
  rosdep update --rosdistro humble
  rosdep install --from-paths src --ignore-src --rosdistro humble -y
fi
for tool in curl cmake git unzip python3; do
  command -v "$tool" >/dev/null || { echo "缺少 $tool，请先运行不带 --skip-apt 的安装。" >&2; exit 1; }
done

# Refuse incompatible existing installs rather than overwriting user libraries.
install_binary() {
  local name="$1" version="$2" version_file="$3" url="$4" archive="$5" digest="$6" library="$7"
  local destination="$ROOT/third_party/$name"
  if [[ -e "$destination" ]]; then
    if [[ -f "$destination/$version_file" && "$(cat "$destination/$version_file")" == "$version" && -f "$destination/lib/$library" ]]; then
      echo "$name $version 已存在。"
      return
    fi
    echo "$destination 已存在但版本或文件不匹配，请先检查并移走该目录。" >&2; exit 1
  fi
  download "$url" "$CACHE/$archive" "$digest"
  mkdir "$STAGING/$name"
  if [[ "$archive" == *.zip ]]; then
    unzip -q "$CACHE/$archive" -d "$STAGING/$name"
    mv "$STAGING/$name/libtorch" "$destination"
  else
    tar -xzf "$CACHE/$archive" --strip-components=1 -C "$STAGING/$name"
    mv "$STAGING/$name" "$destination"
  fi
}
install_binary libtorch '2.5.1+cpu' build-version \
  'https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.5.1%2Bcpu.zip' \
  libtorch-2.5.1-cpu.zip 618ca54eef82a1dca46ff1993d5807d9c0deb0bae147da4974166a147cb562fa libtorch.so
install_binary onnxruntime 1.23.2 VERSION_NUMBER \
  https://github.com/microsoft/onnxruntime/releases/download/v1.23.2/onnxruntime-linux-x64-1.23.2.tgz \
  onnxruntime-1.23.2.tgz 1fa4dcaef22f6f7d5cd81b28c2800414350c10116f5fdd46a2160082551c5f9b libonnxruntime.so

build_sdk() {
  local name="$1" lock_key="$2" digest="$3" commit url source_dir
  read -r url commit < <(python3 - "$lock_key" <<'PY'
import json, sys
item = json.load(open('docs/SENSOR_SDK_UPSTREAM_LOCK.json'))[sys.argv[1]]
print(item['url'].removesuffix('.git'), item['commit'])
PY
)
  source_dir="$ROOT/.deps/sources/$name-$commit"
  if [[ ! -f "$source_dir/.legbot-extracted" ]]; then
    [[ ! -e "$source_dir" ]] || { echo "请检查不完整源码目录：$source_dir" >&2; exit 1; }
    download "$url/archive/$commit.tar.gz" "$CACHE/$name-$commit.tar.gz" "$digest"
    mkdir "$STAGING/$name"
    tar -xzf "$CACHE/$name-$commit.tar.gz" --strip-components=1 -C "$STAGING/$name"
    touch "$STAGING/$name/.legbot-extracted"
    mv "$STAGING/$name" "$source_dir"
  fi
  cmake -S "$source_dir" -B "$ROOT/.deps/build/$name" -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX="$ROOT/third_party/$name/install"
  cmake --build "$ROOT/.deps/build/$name" --parallel "$JOBS"
  cmake --install "$ROOT/.deps/build/$name"
}
build_sdk unitree_sdk2 unitree_sdk2 d9a84be9c5a654aea3d3a707db5776812c0d3b5635e76da356f6d0da3853f7ed
build_sdk Livox-SDK2 livox_sdk2 2e839e9bce66d83cbbc9dade0158133b7547143fb6b09dd210be91d6e271a9bd
OSQP="$ROOT/third_party/osqp"
if [[ ! -e "$OSQP" ]]; then
  git clone --branch v0.6.3 --depth 1 --recurse-submodules --shallow-submodules https://github.com/osqp/osqp.git "$OSQP"
fi
[[ "$(git -C "$OSQP" rev-parse HEAD)" == 0dd00a578cf1c2691c5c379965d504c75bf6cfad ]] || {
  echo 'OSQP 源码不是锁定的 v0.6.3，请先检查该目录。' >&2; exit 1;
}
git -C "$OSQP" submodule update --init --recursive
cmake -S "$OSQP" -B "$ROOT/.deps/build/osqp" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DUNITTESTS=OFF -DCMAKE_INSTALL_PREFIX="$ROOT/third_party/pct/install"
cmake --build "$ROOT/.deps/build/osqp" --parallel "$JOBS"
cmake --install "$ROOT/.deps/build/osqp"
# ROS apt packages are installed for Ubuntu's Python, even when Conda is active.
/usr/bin/python3 -m venv --system-site-packages .venv-humble
.venv-humble/bin/python3 -c 'import numpy, scipy, rclpy'
if $TOMOGRAPHY; then
  .venv-humble/bin/python3 -m pip install -r src/pct_planner/requirements.txt
fi
if ! $SKIP_MODELS || $TOMOGRAPHY; then
  git lfs version
  if ! git config --get filter.lfs.clean >/dev/null; then
    git lfs install --local --skip-repo
  fi
  patterns=''
  if ! $SKIP_MODELS; then patterns='*.pt'; fi
  if $TOMOGRAPHY; then patterns="${patterns:+$patterns,}*.pcd"; fi
  git lfs pull --include="$patterns" --exclude=''
  python3 - "$SKIP_MODELS" "$TOMOGRAPHY" <<'PY'
from pathlib import Path
import sys
files = []
if sys.argv[1] == 'false':
    files += list(Path('src/go2_description/config').rglob('*.pt'))
if sys.argv[2] == 'true':
    files += list(Path('src/pct_planner/src/pcd').glob('*.pcd'))
for p in files:
    with p.open('rb') as f:
        if f.read(128).startswith(b'version https://git-lfs.github.com/spec/v1'):
            raise SystemExit(f'文件仍为 LFS 指针：{p}')
print(f'LFS 文件已就绪（{len(files)} 个）')
PY
fi
printf '\n依赖准备完成。下一步：./build.sh --jobs %s\n' "$JOBS"
