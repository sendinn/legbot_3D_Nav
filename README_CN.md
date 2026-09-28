# Legbot：Unitree GO2 三维自主导航（ROS 2）

> 本分支为 Humble 适配版本；环境准备和本次验证范围见 [Humble 适配说明](docs/HUMBLE_PORT.md)。下文继承的行走、楼梯与真机实验结论来自原 Jazzy 版本，尚不能视为 Humble 上的复现实验结果。

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo Fortress](https://img.shields.io/badge/Gazebo-Fortress-F58113?logo=gazebo)](https://gazebosim.org/docs/fortress/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus)](https://isocpp.org/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](LICENSE)

[English](README.md) | 简体中文 | [项目技术分析](docs/PROJECT_ANALYSIS_CN.md) | [完整运行指南](运行指南.md)

Legbot 是面向 Unitree GO2/GO2 EDU 的 ROS 2 Humble 三维导航工作区。工程将激光—惯性里程计、局部避障、多楼层参考路径、强化学习步态、Gazebo 仿真和带安全门的真机接口组织在同一套启动体系中。

## 项目作用

四足机器人能跨越台阶和不平地形，但导航系统还需要解决“机器人在哪里、前方能否通过、下一步往哪里走、如何把速度变成稳定落足”四个问题。本项目对应地组合了四层能力：

1. FAST-LIO 融合 LiDAR 与 IMU，输出机器人位姿和注册点云；
2. SCAN 根据局部占据图生成无碰、连续且满足速度/加速度约束的 B 样条；
3. PCT 可提供跨楼层全局参考，EGO 可作为另一套局部规划入口；
4. 强化学习控制器把机体速度指令转换为 12 个关节的目标位置和力矩。

```text
LiDAR + IMU ─► FAST-LIO ─► 里程计 + 注册点云 ─► SCAN ─► /cmd_vel
                                                    ▲          │
RViz 目标 / 三维关键点路线 / PCT 全局参考 ──────────┘          ▼
                                              GO2 强化学习控制器
                                                    │
                                             Gazebo / SDK2
```

## 主要功能

- Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress；
- GO2 URDF、ros2_control、Gazebo 硬件插件和 Unitree SDK2 真机接口；
- 仿真雷达、Unitree L1 原装雷达、Livox Mid-360 接入；
- FAST-LIO 激光—惯性定位，不依赖 Gazebo 真值完成默认导航闭环；
- SCAN 滑动占据图、A*、反弹式 B 样条优化、滚动重规划；
- RViz 单目标覆盖（`direct`）、多目标队列（`queue`）和三维关键点路线；
- PCT 多层地图参考规划与 EGO 规划入口；
- TorchScript/ONNX Runtime 双推理后端；
- 真机默认禁用电机命令，提供只读网络与传感器预检；
- 离线依赖、模型、launch 和任务状态机检查。

## 环境与依赖

| 项目 | 要求 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 LTS，当前验证平台为 x86_64 |
| ROS | ROS 2 Humble，Fast DDS |
| 仿真 | Gazebo Fortress |
| 编译 | GCC、CMake、colcon，C++17 |
| Python | Python 3.10、NumPy、SciPy；PCT 制图还使用 Open3D/CuPy |
| 数学/点云 | Eigen3、PCL、OpenCV、yaml-cpp |
| 控制 | ros2_control、ros2_controllers |
| 推理 | LibTorch（C++11 ABI）、ONNX Runtime 1.23.2 |
| 硬件 SDK | Unitree SDK2、Livox SDK2/Driver2 |

新机器（Ubuntu 22.04 x86_64）在仓库目录用普通用户执行：

```bash
./install.sh
./build.sh
source tools/env.sh
```

`install.sh` 自动准备 ROS Humble 软件源与 Desktop、Fortress、系统依赖、
LibTorch/ONNX Runtime、固定版本 SDK、OSQP、Python 环境和 Git LFS 策略文件。
系统安装步骤会调用 `sudo`，其他依赖保存在工作区内。重复执行会复用下载缓存和编译产物。
`build.sh` 默认单包 2 线程、各包顺序编译，可用 `./build.sh --jobs 4` 调整，
也支持 `--packages-select`、`--packages-up-to`。

默认支持已有 tomogram 的 CPU 规划；需要从 PCD 制图时执行
`./install.sh --with-tomography`，运行制图还需可用的 CUDA 12 驱动/运行环境。
详细选项与手动依赖说明见 [Humble 安装说明](docs/HUMBLE_PORT.md)。

SDK 固定版本见 [`docs/SENSOR_SDK_UPSTREAM_LOCK.json`](docs/SENSOR_SDK_UPSTREAM_LOCK.json)，模型来源、输入维度和 SHA-256 见 [`docs/GO2_POLICY_MODELS_LOCK.json`](docs/GO2_POLICY_MODELS_LOCK.json)。

## 目录说明

| 路径 | 内容 |
| --- | --- |
| `src/legbot_bringup` | 总启动入口、适配器、世界、RViz 配置 |
| `src/go2_description` | GO2 模型、控制配置、策略权重 |
| `src/fast_lio` | 激光—惯性里程计 |
| `src/scan_planner` | 局部占据图、搜索、轨迹优化、命令适配 |
| `src/pct_planner` | 多层层析图、全局规划和轨迹优化 |
| `src/ego_planner` | EGO 局部规划器 |
| `src/rl_quadruped_controller` | 策略推理、状态机、关节控制 |
| `src/gz_quadruped_hardware` | Gazebo ros2_control 插件 |
| `src/hardware_unitree_sdk2` | GO2 真机接口和 L1 雷达桥 |
| `tools` | 编译、环境、离线检查、真机只读预检 |
| `tests` | 离线检查和单独执行的仿真启动检查 |
| `docs` | 项目分析、架构状态、上游版本与模型锁定记录 |

## 编译与离线检查

```bash
# 使用当前工作区已创建的本地 humble 分支
cd /path/to/legbot_3D_Nav
git switch humble

./build.sh
source tools/env.sh
./tools/check_offline.sh
```

离线检查不会启动 Gazebo，也不会连接真机。通过时最后一行应为：

```text
PASS offline checks; no simulation or robot experiment was started by this check
```

## 组合地形仿真快速启动

推荐在工作区根目录一键启动：

```bash
./simulation.sh
# 无界面：./simulation.sh --headless
```

脚本自动加载环境、等待控制器激活并启动完整导航入口；等待“导航数据已就绪”后
在 RViz 使用 **2D Goal Pose**。Ctrl+C 自动关闭本次启动的进程。
启动时会分别询问 Gazebo 和 RViz 是否使用 GPU（y/n）。WSL 上 Gazebo 默认 CPU 软件渲染，
RViz 默认 GPU；回车接受默认值。Gazebo 的选择包含雷达渲染，无界面时仍需选择。
关闭 RViz 时跳过其询问；非交互运行和 `--check` 使用默认值。
可直接指定：`./simulation.sh --gazebo-rendering software --rviz-rendering gpu`。
两者渲染环境独立，Gazebo 的软件渲染设置不会再传给 RViz。
GPU 选项取消 CPU 强制设置，实际渲染设备由系统驱动决定；本机 WSL 的 Gazebo GPU 雷达此前曾崩溃。

详细注释、日志路径和参数见 `simulation.sh --help` 及 [运行指南](运行指南.md)。
下方六终端流程用于分模块调试，不要与一键入口同时运行。

每个命令使用独立终端；新终端先进入工作区并执行 `source tools/env.sh`。

```bash
# 终端 1：Gazebo、GO2 和默认 moe_cts_77k 策略
ros2 launch legbot_bringup simulation.launch.py gui:=true

# 终端 2：站立和导航闭环状态机
ros2 launch legbot_bringup go2_demo_control.launch.py

# 终端 3：FAST-LIO
ros2 launch legbot_bringup fastlio.launch.py use_sim_time:=true \
  config:=$PWD/install/legbot_bringup/share/legbot_bringup/config/fastlio_sim.yaml

# 终端 4：SCAN
ros2 launch legbot_bringup scan.launch.py use_sim_time:=true

# 终端 5：RViz
ros2 launch legbot_bringup scan_rviz.launch.py

# 终端 6：RViz 航点入口
ros2 launch legbot_bringup scan_waypoints.launch.py
```

等待 FAST-LIO 输出 `IMU Initial Done`，并等待状态机输出 `FAST-LIO ready: enabling SCAN waypoint mission`，随后在 RViz 中使用 **2D Goal Pose**。默认 `direct` 模式会用新目标替换旧目标；需要连续执行多个点时切换为 `queue`。

默认唯一世界为 `Building.sdf`：机器人从西侧平地的 30 个圆柱和 3 道拱门障碍区出发，
再连续进入东侧 Building 多楼层楼梯区。无界面运行、关键点录制、PCT、EGO、真值对比和
停止顺序见[完整运行指南](运行指南.md)。

要自动执行“当前位置 → 平地避障 → Building 入口 → PCT 跨层全局路径 → 顶层终点”，
不要再启动上面的六个独立终端，直接运行：

```bash
ros2 launch legbot_bringup pct_cross_floor_demo.launch.py
```

该长距离跨层演示默认用 Gazebo 真值里程计消除仿真 FAST-LIO 的累计高度漂移；PCT 的
105 点跨层全局路线、SCAN 局部避障和 GO2 控制链保持不变。需要专门测试定位集成时可加
`navigation_source:=fastlio`。RViz 会自动使用楼梯场景配置打开。

## 策略配置

| `policy_profile` | 后端与输入 | 用途 |
| --- | --- | --- |
| `moe_cts_77k` | ONNX，`1×450`，10 帧 term-major 历史 | 默认；平地导航和楼梯动力学 |
| `go2_cts` | TorchScript，单帧 45 维 | 平地对照 |
| `mjlab_flat` | ONNX，单帧 45 维 | Unitree Velocity Flat 对照 |

策略观测由机体角速度、重力方向、速度命令、关节位置误差、关节速度和上一时刻动作组成。策略输出 12 维动作，经缩放后成为关节目标位置，再由 PD 环控制。

## 真机安全入口

真机 launch 默认 `enable_commands:=false`。首次连接只执行：

```bash
./tools/go2_real_preflight.sh <网卡名>
```

脚本只读检查 `192.168.123.x` 网络、12 个关节状态、机身 IMU、L1 内置 IMU以及点云的 `x/y/z/intensity/time/ring` 字段，不发布 `LowCmd`。放行电机命令前必须现场确认急停、吊架、网卡、时间戳、外参、静止漂移和低速零指令。

## 算法概要

FAST-LIO 使用迭代误差状态卡尔曼滤波，将 IMU 传播与点到平面残差紧耦合。SCAN 对点云进行射线更新，维护概率占据图；初始轨迹的碰撞段由 A* 修复，再优化 B 样条控制点：

$$J=\lambda_sJ_{smooth}+\lambda_cJ_{collision}+\lambda_fJ_{feasibility}+\lambda_rJ_{reference}.$$

策略动作到关节控制的基本关系为：

$$q_d=q_0+s_a a,\qquad \tau=K_p(q_d-q)-K_d\dot q,$$

最后按各关节力矩上限裁剪。详细状态定义、占据概率、A*、B 样条、PCT、EGO、控制链和参数解释见[项目技术分析](docs/PROJECT_ANALYSIS_CN.md)。

## 当前验证边界

- 已完成默认策略的 Gazebo 平地动力学与 FAST-LIO + SCAN 导航闭环；
- 已完成 `moe_cts_77k` 八级楼梯底层动力学测试；
- Gazebo 默认导航输入不使用真值里程计；
- 真机接口和只读传感器预检已经接入，但本仓库不宣称电机闭环已经完成现场验收；
- Unitree L1/机身外参、现场网络和定位漂移仍应在每台机器人上重新检查。

## 许可证

本项目原创集成代码使用 [Apache License 2.0](LICENSE)。仓库内的上游代码和预训练模型继续适用各自许可证，其中 FAST-LIO 与 PCT 含 GPL-2.0 代码。分发或商用前请阅读 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 致谢

项目基于 FAST-LIO、SCAN-Planner、EGO-Planner、PCT Planner、Unitree SDK2、Livox SDK/Driver2、`quadruped_ros2_control`、LibTorch 与 ONNX Runtime。对应上游版本、来源和许可记录保留在 `docs/`、各软件包许可证和 `THIRD_PARTY_NOTICES.md` 中。
