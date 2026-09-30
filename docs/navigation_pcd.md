# 使用已有 PCD 导航

在项目根目录运行；先用 Ctrl+C 退出旧的 simulation.sh 或 mapping.sh：

    ./navigation.sh maps/20260929-153543-3175654/map.pcd

不传路径会选 maps/ 下最近保存的 map.pcd。也支持绝对路径。

## 操作顺序

1. 脚本读取 PCD，生成分层可通行地图缓存，然后启动 Gazebo、机器人控制、SCAN、地图导航节点和 RViz。
2. RViz Fixed Frame 自动设为 navigation_map。Loaded PCD 显示原始地图；Walkable surfaces 显示可通行表面。
3. 选择 **3D Pose Estimate**，在已有地图上点机械狗当前所在的地面，按住拖动表示朝向，再松开。工具自动加 0.5 米作为机身高度估计。需要根据 Gazebo 中机器人的实际位置在地图里选对应位置；配准前机器人模型可能暂时不显示。
4. 等待终端显示 **ALIGNED**。节点将当前激光扫描与已有点云进行 ICP 配准，检查匹配比例和残差；失败时重新选择位置和朝向。
5. 选择 **3D Nav Goal**，点击目标楼层可走的地面，拖动后松开。成功显示 **GOAL SENT**，PCT 路线经坐标转换发送给 SCAN，控制器开始执行。
6. 到达显示 **ARRIVED**，可继续选下一目标。Ctrl+C 停止本次进程。

当前目标朝向不作为终点转向约束，路线末端朝向由路径切线决定。点击时的高度来自拾取到的点云表面，不能点空白区域、墙面或空中；目标会在同层 0.6 米范围内寻找可通行格。换层视角可使用 RViz 的 Views / Orbit，避免点击到遮挡的上层楼板。

## 数据与坐标

- 支持 ASCII、binary、binary_compressed PCD，读取 XYZ，过滤无效点。
- CPU 根据地面/顶面、坡度、台阶高度、净空和障碍膨胀生成 PCT 兼容的分层地图；未观察到的区域保持不可通行。该转换无需安装 Open3D/CuPy。
- 缓存保存在 data/navigation/<内容和参数哈希>/，包括 points.npy、map.pickle、map.json、navigation.rviz；同一地图和参数重复启动会复用缓存。
- 原始点云话题 /navigation/map，可通行面 /navigation/traversability，坐标 navigation_map。
- 当前仿真里程计和实时点云坐标为 odom。成功配准建立 odom -> navigation_map 的 TF；PCT 在 navigation_map 内规划，输出 /pct_path 和 /scan/goal 前转换到 odom。
- last_alignment.json 仅用于诊断，不会在下一次启动时自动使用。每次启动需要重新初始定位。
- 状态话题 /navigation/state；本次日志位于 log/navigation/<时间戳>/。
- 保留原有仿真互斥锁，防止多个实例控制同一机器人。

## 参数

    ./navigation.sh /path/to/map.pcd --check
    ./navigation.sh /path/to/map.pcd --prepare-only
    ./navigation.sh /path/to/map.pcd --resolution 0.15 --slice-height 0.5
    ./navigation.sh /path/to/map.pcd --world /path/to/corresponding_world.sdf
    ./navigation.sh /path/to/map.pcd --software-rendering

--check 检查 PCD、ROS 包和原生 PCT 库；--prepare-only 只生成缓存，不启动仿真。另有 --headless、--no-gazebo-gui、--no-rviz、测试用 --duration 秒。

## 适用范围和限制

本版对接现有 Gazebo 仿真，默认 Building.sdf。PCD 必须与正在运行的仿真世界对应；加载点云不会自动创建或替换 Gazebo 的碰撞场景。

仿真使用 ground-truth 里程计配合一次初始 ICP 对齐。ICP 是局部配准，需要接近真实位置和朝向的初值；重复楼层或特征不足区域可能存在歧义，ALIGNED 后应检查地图与实时扫描是否重合。它不是全局自动重定位或持续消除里程计漂移的真机定位系统。真机需要另外接入与已有地图匹配的持续定位、传感器外参和运动控制接口，本脚本没有提供真机启动模式。

楼梯必须已被扫描，且地图存在连续可通行连接。地图稀疏、漂移、楼板遮挡、净空不足或台阶超出模型阈值都会导致拒绝目标或无路径。没有路径不会用直线替代。

## 已验证

- PCD 三种编码、损坏文件拒绝、分层地图、未知区/低净空阻挡、ICP 成功与失败、缓存失效：tests/check_navigation_offline.py。
- 隔离 ROS 域的合成地图集成测试：tests/check_navigation_ros.py，覆盖地图发布、未定位目标拒绝、错误配准拒绝、正确配准、原生 PCT 路径、SCAN 输入坐标、完成回执。
- 已对 maps/20260929-153543-3175654/map.pcd 生成缓存。
- 已完成 Gazebo 启动、控制器起立、PCD 导航节点和 RViz OpenGL 初始化的运行测试；尚未验证机械狗实际走完路线或人工核对窗口画面。

## Gazebo 黑屏启动排查

脚本先单独启动仿真服务，控制器就绪后再打开 Gazebo 窗口。机器人创建超时会立即报告错误并清理本次进程，避免一直显示空窗口。日志分为 simulation.log、gazebo_gui.log 和 rviz.log。

如果 Gazebo 窗口仍异常，可运行 ./navigation.sh --no-gazebo-gui，仅显示 RViz，仿真物理和激光仍运行。渲染驱动问题可尝试 --software-rendering。

### NVIDIA 驱动版本冲突（2026-09-30）

本机发现已加载内核模块 590.48.01 与 NVML 库 595.91 不一致，导致 EGL 初始化失败后 Gazebo 段错误。navigation.sh 默认检测 nvidia-smi 的版本冲突并切换 Mesa 软件渲染，显式设置 Mesa GLX/EGL 提供者；有 DISPLAY 时使用 GLX，无桌面时使用 EGL。--gpu-rendering 可显式强制 GPU，不会自动回退。此次没有修改系统驱动或重启服务器。

控制器启动时先等待 spawner 激活完成，再查询服务确认状态，避免加载阶段持续查询。已验证软件渲染下机器人起立、实时激光点云输出和 RViz OpenGL 4.5 初始化。软件渲染会增加 CPU 占用。
