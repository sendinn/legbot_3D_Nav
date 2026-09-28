# ROS 2 Humble 适配

本地分支 `jazzy` 保留原 `ROS2` 分支代码；`humble` 基于同一提交进行适配。
目标环境为 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress（Ignition Gazebo 6）、Python 3.10。
两个分支的 `build/`、`install/`、虚拟环境和二进制依赖不可混用；切换发行版推荐使用独立 worktree。
本次操作不推送远端分支。

## 新机器两步安装

在 Ubuntu 22.04 x86_64 上，用普通用户在当前 `humble` 工作区执行：

```bash
./install.sh
./build.sh
source tools/env.sh
./tools/check_offline.sh
```

无需预先安装 ROS；安装脚本按 [ROS 官方流程](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst)
检查软件源，缺失时获取官方 `ros2-apt-source` 安装包，再安装 Humble Desktop 与依赖。
需要联网和 sudo 权限；不要用 sudo 执行整个脚本。当前不支持 ARM64 或 Ubuntu 24.04。

| 选项 | 作用 |
| --- | --- |
| `./install.sh --jobs 4` | 调整 SDK/OSQP 编译并行度，默认 2 |
| `./install.sh --skip-apt` | 系统和 ROS 依赖已准备好时跳过 apt/rosdep |
| `./install.sh --skip-models` | 跳过已有策略文件下载 |
| `./install.sh --with-tomography` | 额外装 Open3D/CuPy、拉取 PCD；运行需 CUDA 12 |
| `./build.sh --jobs 4` | 调整项目单包编译并行度 |
| `./build.sh --packages-up-to scan_planner` | 仅构建指定包及其工作区依赖 |

下载缓存、SDK 源码和依赖构建在 `.deps/`；运行库在 `third_party/`。
推理库与 SDK 压缩包有 SHA-256 校验；OSQP 固定为 0.6.3 的 commit。
已有不同版本的推理库不会被自动覆盖。安装脚本不配置 NVIDIA 驱动或 CUDA Toolkit。
普通使用不必手动执行下述步骤，以下保留为依赖与排障参考。

## 系统依赖（手动参考）

安装 ROS 2 Humble Desktop 和其软件源后，在本仓库目录执行：

```bash
sudo apt update
sudo apt install git-lfs ros-dev-tools python3-rosdep python3-venv python3-pip \
  python3-numpy python3-scipy pybind11-dev libeigen3-dev libpcl-dev \
  libopencv-dev libyaml-cpp-dev libcap-dev \
  ros-humble-ros-gz ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-pcl-ros ros-humble-cv-bridge ros-humble-tf2-sensor-msgs \
  ros-humble-backward-ros ros-humble-rmw-fastrtps-cpp ros-humble-gtsam \
  ros-humble-interactive-markers libignition-gazebo6-dev libignition-plugin-dev
source /opt/ros/humble/setup.bash
# 首次使用 rosdep 时执行 sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
python3 -m venv --system-site-packages .venv-humble
git lfs install --local
git lfs pull --include="*.pt" --exclude=""
```

`build.sh`（内部调用 `tools/build.sh`）和 `tools/env.sh` 拒绝已加载其他 ROS 发行版的终端。可选的
`.deps/humble/opt/ros/humble` 是本地解压开发依赖的前缀；正常 apt 安装不需要它。

## 非 ROS 依赖

所有大型下载和编译产物放在 Git 忽略的 `third_party/` 下。

- LibTorch：本次使用官方 CPU **2.5.1，C++11 ABI**，解压为 `third_party/libtorch`。
  [下载](https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.5.1%2Bcpu.zip)。
- ONNX Runtime：官方 Linux x64 **1.23.2**，解压并去掉版本目录，放入
  `third_party/onnxruntime`。[下载](https://github.com/microsoft/onnxruntime/releases/download/v1.23.2/onnxruntime-linux-x64-1.23.2.tgz)。
- Unitree SDK2 与 Livox SDK2：仓库和 commit 见
  [SDK 锁定文件](SENSOR_SDK_UPSTREAM_LOCK.json)，分别放入 `third_party/unitree_sdk2`
  和 `third_party/Livox-SDK2`，用 CMake 编译安装到各自的 `install/`。
- PCT 使用 GTSAM 4.2、pybind11 和 **OSQP 0.6.3**；不能直接换成 OSQP 1.x。

```bash
mkdir -p third_party
git clone https://github.com/unitreerobotics/unitree_sdk2.git third_party/unitree_sdk2
git -C third_party/unitree_sdk2 checkout 9754cd153af3da471b0fe5f3aa535e426fb11db3
git clone https://github.com/Livox-SDK/Livox-SDK2.git third_party/Livox-SDK2
git -C third_party/Livox-SDK2 checkout 08f523c930b2f0ba1e98a6afaa8d7476bf479908
# 对已按锁定 commit 获取的两个 SDK 分别执行
for sdk in unitree_sdk2 Livox-SDK2; do
  cmake -S "third_party/$sdk" -B "third_party/$sdk/build" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF \
    -DCMAKE_INSTALL_PREFIX="$PWD/third_party/$sdk/install"
  cmake --build "third_party/$sdk/build" -j2
  cmake --install "third_party/$sdk/build"
done

git clone --branch v0.6.3 --depth 1 --recurse-submodules --shallow-submodules \
  https://github.com/osqp/osqp.git third_party/osqp
cmake -S third_party/osqp -B third_party/osqp/build \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DUNITTESTS=OFF \
  -DCMAKE_INSTALL_PREFIX="$PWD/third_party/pct/install"
cmake --build third_party/osqp/build -j2
cmake --install third_party/osqp/build
```

使用已有 tomogram 做 PCT 路径规划不需要 Open3D/CuPy。
从点云生成 tomogram 才需要额外安装 `src/pct_planner/requirements.txt` 中的依赖，
其中 CuPy 对应 CUDA 12；有 NVIDIA/CUDA 环境时再配置这一部分。

## 编译和运行

```bash
./build.sh
source tools/env.sh
./tools/check_offline.sh
IGN_IP=127.0.0.1 ros2 launch legbot_bringup simulation.launch.py gui:=true
```

上述 `IGN_IP` 将本机仿真通信绑定回环；本次多网卡环境中默认地址导致实体创建服务超时。
分布式 Gazebo 通信请改为对应网卡地址。

`./build.sh --jobs 2` 可调整单包编译并行度，内存不足时使用 `--jobs 1`。
`PCT_PYTHON=/usr/bin/python3` 可替代 `.venv-humble/bin/python3`，但必须是 Python 3.10。
额外的制图依赖检查：`python3 tests/check_dependencies_offline.py --tomography`。
仿真启动回归检查：`python3 tests/check_humble_sim.py`。此检查启动独立 DDS 域的无界面仿真，
确认控制器激活与关节状态后关闭，不发送行走命令。
导航和真机参数见 [运行指南](../运行指南.md)。真机仍默认 `enable_commands:=false`。

## 主要适配点

- Gazebo 插件改为 Fortress 的 `ignition::gazebo` API、插件名称、资源路径和消息类型。
- Humble ros2_control 显式导入并激活仿真硬件；从 robot_state_publisher 获取 URDF。
- 控制器改用 Humble 的 `get_value()` / `set_value()` 接口，并兼容 Humble 提前声明参数的行为，避免激活时重复声明异常。
- 仿真力矩限制从 SDF 关节轴读取；真机位置/力矩限制从 URDF 读取，缺少有效限制即初始化失败。
- 真机 controller_manager 显式传入 robot_description，并适配私有描述话题。
- SCAN/EGO 使用 Humble 的 cv_bridge 头文件；Livox 固定 ROS 2 包清单，允许直接 colcon 构建。
- Python 扩展面向 3.10 重编译；PCT 允许使用 ROS 安装的 GTSAM，去掉对固定 METIS 路径的要求。
- 离线检查报告只记录本次执行的检查，不再把旧 Jazzy 运行结果写成当前已通过。

## 验证范围

2026-09-28，本机 Ubuntu 22.04 / ROS 2 Humble / Fortress 6.18.0：

- 全部 **14 个 ROS 包编译通过**。
- 全部 14 个 bringup launch 描述检查通过，GO2 URDF/SDF、世界和模型来源校验通过。
- 三套 TorchScript、三套 ONNX 策略的输入/输出与推理契约检查通过。
- 急停、目标停止/恢复/旧路径拒绝、PCT 任务坐标变换检查通过。
- PCT 原生扩展加载通过；已有 building2_9 tomogram 规划得到 105 点跨楼层路径。
- Fortress 无界面启动测试通过：joint_state_broadcaster、imu_sensor_broadcaster、
  rl_quadruped_controller 均 active，默认 ONNX 策略预热完成，收到随仿真时间更新的
  12 个有效关节位置。该测试关闭激光雷达，以隔离检查仿真硬件和控制器启动。

本地日志：`log/offline/validation.json`、`log/humble_sim_smoke.log`。
未验证：Humble 下完整激光定位/导航闭环、行走/楼梯动力学、真机联调、CUDA 制图。
历史 Jazzy 的相关实验结果不能自动推断为 Humble 的复现结果。

父仓库的 `.gitmodules` 指向 `humble`。在远端发布这个分支之前，其他克隆不能通过
`git submodule update --remote` 获取本次适配；本次仅整理本地分支和提交。
