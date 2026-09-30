# 键盘控制机械狗扫图（仿真）

在工程根目录运行：

```bash
./mapping.sh
```

先退出同一工作区的 `simulation.sh`。脚本使用同一把锁，防止两个控制入口同时运行。
它会启动 Gazebo、雷达、FAST-LIO 和建图 RViz，自动起身，定位就绪后进入行走模式。
不启动 SCAN、航点任务或真机驱动。键盘操作需要终端焦点；远程运行请使用 SSH 的 `-t` 选项。

| 按键 | 功能 |
| --- | --- |
| W / S | 前进 / 后退 |
| A / D | 左 / 右横移 |
| Q / E | 左 / 右转向 |
| 空格 / X | 速度归零 |
| P | 停止移动并保存当前地图，完成后可继续扫图 |
| Esc / Ctrl+C | 停止移动，尝试保存地图，再退出本次仿真 |

持续按移动键；终端不提供松键事件，最后一次按键后 0.5 秒自动归零。
默认前后速度 0.25 m/s，横移最大 0.25 m/s，转向 0.4 rad/s。
这是手动扫图，需自行控制行走路线和避障。

默认地图目录为 `maps/<时间戳-PID>/`：

- `map.pcd`：FAST-LIO 累计的三维点云，默认坐标系为 `odom`。
- `map.json`：保存时间、点数和坐标系。
- `fastlio.yaml`：本次实际使用的配置。
- `map.pending.pcd`：仅在保存进行中或失败时可能存在。保存成功并通过格式检查后替换 `map.pcd`。

每次 P 会更新本次目录中的地图。退出时再保存最终地图；如失败，会保留上一次成功保存的地图并报告错误。
不同启动使用不同目录；自定义输出目录必须尚不存在。
地图不是二维 Nav2 的 PGM/YAML 栅格地图。FAST-LIO 本身没有回环优化，长距离累计的定位漂移仍可能影响地图质量。

常用命令：

```bash
./mapping.sh --output maps/office_scan
./mapping.sh --speed 0.2 --turn-speed 0.3
./mapping.sh --no-gazebo-gui
./mapping.sh --headless --gpu-rendering
./mapping.sh --software-rendering
./mapping.sh --check
```

`--duration 30` 会在建图就绪后运行 30 秒，保存并退出；无交互终端时为静止采样，不会自动巡航。
日志在 `log/mapping/<时间戳-PID>/`。DDS 域沿用 `LEGBOT_SIM_DOMAIN_ID`，默认 178，且限定本机通信。
启动超时沿用 `LEGBOT_SIM_START_TIMEOUT`，默认 120 秒。
无需重新编译，脚本直接使用已构建的仿真与 FAST-LIO。


## NVIDIA 驱动版本冲突

若 nvidia-smi 报 Driver/library version mismatch，mapping.sh 会提示并自动将 Gazebo/RViz 切换到 Mesa 软件渲染。有 DISPLAY 时使用桌面 GLX 后端。此设置只作用于本次子进程，不修改系统驱动。软件渲染可能较慢。2026-09-30 已验证完整启动、FAST-LIO 就绪和 PCD 保存（34146 点）；静止采样测试文件放在 log/mapping/20260930-105141-600363/test_map，避免影响自动选择用户地图。
