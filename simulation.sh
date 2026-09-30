#!/usr/bin/env bash
# 一键启动 GO2 仿真导航：Gazebo → FAST-LIO → SCAN → 步态控制。
# 首次使用：./install.sh && ./build.sh
# 日常使用：./simulation.sh；无图形界面：./simulation.sh --headless
# 本脚本仅启动仿真，不启动 real.launch.py，也不主动发送导航目标。
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
GUI=true
RVIZ=true
CHECK=false
CROSS_FLOOR=false
INTERACTIVE_3D=false
GOAL_X=2.0
GOAL_Y=-3.0
GOAL_Z=4.5
GAZEBO_RENDERING=auto
RVIZ_RENDERING=auto
while (($#)); do
  case "$1" in
    --3d) CROSS_FLOOR=true; INTERACTIVE_3D=true; shift ;;
    --upstairs) CROSS_FLOOR=true; INTERACTIVE_3D=false; shift ;;
    --goal3d)
      [[ $# -ge 4 ]] || { echo '--goal3d 需要 X Y Z 三个坐标' >&2; exit 2; }
      CROSS_FLOOR=true; INTERACTIVE_3D=false; GOAL_X="$2"; GOAL_Y="$3"; GOAL_Z="$4"; shift 4 ;;
    --headless) GUI=false; RVIZ=false; shift ;;
    --no-rviz) RVIZ=false; shift ;;
    --no-gazebo-gui) GUI=false; shift ;;
    --check) CHECK=true; shift ;;
    --software-rendering) GAZEBO_RENDERING=software; RVIZ_RENDERING=software; shift ;;
    --gpu-rendering) GAZEBO_RENDERING=gpu; RVIZ_RENDERING=gpu; shift ;;
    --gazebo-rendering|--rviz-rendering)
      [[ $# -ge 2 && ( "$2" == gpu || "$2" == software ) ]] || {
        echo "$1 需要 gpu 或 software" >&2; exit 2;
      }
      if [[ "$1" == --gazebo-rendering ]]; then GAZEBO_RENDERING="$2"; else RVIZ_RENDERING="$2"; fi
      shift 2 ;;
    -h|--help)
      cat <<'HELP'
用法：./simulation.sh [--headless] [--no-gazebo-gui] [--no-rviz] [--software-rendering] [--check]
--3d  RViz 工具栏 3D Nav Goal：按住楼层点云、拖动方向、松开发送目标
--upstairs  启动 PCT 跨楼层模式，默认楼上目标 [2, -3, 4.5]
--goal3d X Y Z  指定 building_pct 楼栋地图中的三维目标（米），自动启用跨层模式
跨层模式使用 Gazebo 真值定位和预建 building2_9 地图，按楼梯路径导航。
坐标属于 building_pct；Gazebo 世界坐标为 [X+13, Y, Z]，不是 FAST-LIO odom。
目标需落在该地图可通行区域。启动后执行一次任务，修改目标需重新启动。
不带跨层参数时启动 Gazebo GUI、FAST-LIO、SCAN、站立状态机、航点入口和 RViz。
--headless  关闭 Gazebo GUI 和 RViz，仍保留激光雷达与完整导航链路
--no-gazebo-gui  关闭 Gazebo 窗口，保留 RViz 和完整导航（低负载调试）
--no-rviz   仅关闭 RViz
--gazebo-rendering gpu|software  单独指定 Gazebo 渲染，跳过对应询问
--rviz-rendering gpu|software    单独指定 RViz 渲染，跳过对应询问
--software-rendering / --gpu-rendering  同时指定两者，兼容旧参数
未指定时分别询问；WSL 的 Gazebo 默认软件渲染，RViz 默认 GPU。
非交互终端使用默认值；--check 不询问；关闭 RViz 时不询问 RViz。
无界面的 Gazebo 仍需渲染激光雷达，因此仍会询问 Gazebo。
--check     只检查环境和启动文件，不启动仿真
Ctrl+C     先停止导航进程，再停止 Gazebo，并清理本次启动的进程组
日志       log/simulation/<时间戳-PID>/，每个模块单独一个文件
环境变量   LEGBOT_SIM_DOMAIN_ID：独立仿真 DDS 域，默认 178
           LEGBOT_SIM_START_TIMEOUT：控制器启动超时秒数，默认 120
HELP
      exit 0 ;;
    *) echo "未知参数：$1；使用 --help 查看用法。" >&2; exit 2 ;;
  esac
done
# 加载完整环境（包括 .deps 中的 ROS 消息包、推理库和 Python 虚拟环境）。
# ROS 的 setup 脚本可能读取未定义变量，source 期间暂时关闭 nounset。
set +u
source "$ROOT/tools/env.sh"
set -u

# 使用独立 DDS 域及本机通信，避免 /control_input 与其他机器人实例混用。
# 如需另开终端调试话题，需同样 source tools/env.sh，并设置这两个 ROS 变量。
export ROS_DOMAIN_ID="${LEGBOT_SIM_DOMAIN_ID:-178}"
export ROS_LOCALHOST_ONLY=1
# 本机多网卡/WSL 环境中显式绑定回环，避免 Gazebo 创建实体服务超时。
export IGN_IP=127.0.0.1
export IGN_PARTITION="legbot_navigation_${ROS_DOMAIN_ID}_$$"
export PYTHONUNBUFFERED=1
# 单独选择渲染方式。无界面的 Gazebo 仍需渲染 GPU 雷达，不能跳过它。
# read 必须在下方 Python heredoc 接管 stdin 之前执行。
# 显式命令行参数优先；非交互任务和 --check 使用默认值，不阻塞等待输入。
choose_rendering() {
  local label="$1" default="$2" variable="$3" answer hint
  if [[ "${!variable}" != auto ]]; then return; fi
  printf -v "$variable" '%s' "$default"
  if $CHECK || [[ ! -t 0 ]]; then return; fi
  hint='Y/n'
  if [[ "$default" == software ]]; then hint='y/N'; fi
  while true; do
    if ! read -r -p "$label 是否使用 GPU 渲染？[$hint]（n = CPU 软件渲染）：" answer; then
      echo '输入结束，取消启动。' >&2; exit 1
    fi
    case "${answer,,}" in
      '') return ;;
      y|yes) printf -v "$variable" '%s' gpu; return ;;
      n|no) printf -v "$variable" '%s' software; return ;;
      *) echo '请输入 y 或 n，也可直接回车使用默认值。' ;;
    esac
  done
}
gazebo_default=gpu
if [[ "$(cat /proc/sys/kernel/osrelease)" == *[Mm]icrosoft* ]]; then
  gazebo_default=software
fi
choose_rendering 'Gazebo（物理仿真窗口和雷达）' "$gazebo_default" GAZEBO_RENDERING
if $RVIZ; then
  choose_rendering 'RViz（地图和点云显示）' gpu RVIZ_RENDERING
fi
echo "Gazebo 渲染：$GAZEBO_RENDERING"
if $RVIZ; then echo "RViz 渲染：$RVIZ_RENDERING"; else echo 'RViz：未启动'; fi
# 此处不全局 export LIBGL_ALWAYS_SOFTWARE/GALLIUM_DRIVER。
# Python 在启动各自进程时构造独立环境，防止 Gazebo 的 CPU 配置传给 RViz。

# flock 锁在整个脚本生命周期内有效；异常退出后内核也会释放。
# 只防止本工作区重复运行该脚本，不终止用户已经运行的其他程序。
mkdir -p "$ROOT/log"
exec 9>"$ROOT/log/simulation.lock"
flock -n 9 || { echo '本工作区已有 simulation.sh 在运行，请先退出它。' >&2; exit 1; }

# 使用 Python 管理进程和 ROS 服务等待，避免依靠固定 sleep 猜测控制器已启动。
# 每个 ros2 launch 独占进程组，退出时仅清理本脚本记录的组，不使用全局 pkill。
exec python3 - "$GUI" "$RVIZ" "$CHECK" "$GAZEBO_RENDERING" "$RVIZ_RENDERING" "$CROSS_FLOOR" "$GOAL_X" "$GOAL_Y" "$GOAL_Z" "$INTERACTIVE_3D" <<'PY'
import datetime
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path.cwd() / 'tools'))
from simulation_cleanup import cleanup_partition

import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers

gui, rviz, check = (arg == 'true' for arg in sys.argv[1:4])
gazebo_rendering, rviz_rendering = sys.argv[4:6]
cross_floor = sys.argv[6] == 'true'
interactive_3d = sys.argv[10] == 'true'
try:
    goal = tuple(float(value) for value in sys.argv[7:10])
    if len(goal) != 3 or not all(math.isfinite(value) for value in goal):
        raise ValueError
except ValueError:
    sys.exit('--goal3d 坐标必须为三个有限数字')
try:
    domain = int(os.environ['ROS_DOMAIN_ID'])
    timeout = float(os.environ.get('LEGBOT_SIM_START_TIMEOUT', '120'))
    if not 0 <= domain <= 232 or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError
except ValueError:
    sys.exit('DDS 域必须是 0～232 的整数，启动超时必须是有限正数。')
share = Path(get_package_share_directory('legbot_bringup'))
launches = {
    'crossfloor': 'pct_cross_floor_demo.launch.py',
    'simulation': 'simulation.launch.py',
    'control': 'go2_demo_control.launch.py',
    'fastlio': 'fastlio.launch.py',
    'scan': 'scan.launch.py',
    'waypoints': 'scan_waypoints.launch.py',
    'rviz': 'scan_rviz.launch.py',
}
config = share / 'config/fastlio_sim.yaml'
for path in [config, *(share / 'launch' / name for name in launches.values())]:
    if not path.is_file():
        sys.exit(f'缺少 {path}，请先执行 ./build.sh')
if cross_floor:
    result = subprocess.run(['ros2', 'run', 'pct_planner', 'pct_plan', '--check-libraries'])
    if result.returncode:
        sys.exit('PCT 依赖检查失败，请检查 pct_planner 构建')
if check:
    print('PASS 环境、控制器消息和导航启动文件检查；未启动仿真。')
    sys.exit(0)

log_dir = Path('log/simulation') / (datetime.datetime.now().strftime('%Y%m%d-%H%M%S') + f'-{os.getpid()}')
log_dir.mkdir(parents=True)
(log_dir / 'rendering.txt').write_text(
    f'ROS_DOMAIN_ID={domain}\nGazebo={gazebo_rendering}\nRViz={rviz_rendering if rviz else "disabled"}\n')
processes = []
node = None

def rendering_environment(mode):
    env = os.environ.copy()
    # GPU 模式取消从父终端继承的 CPU 强制设置；实际硬件由图形驱动选择。
    # 不覆写 DISPLAY、WAYLAND_DISPLAY 或用户的其他驱动配置。
    env.pop('LIBGL_ALWAYS_SOFTWARE', None)
    env.pop('GALLIUM_DRIVER', None)
    if mode == 'software':
        env['LIBGL_ALWAYS_SOFTWARE'] = '1'
        env['GALLIUM_DRIVER'] = 'llvmpipe'
    return env

def start(name, *arguments):
    path = log_dir / f'{name}.log'
    env = None
    if name in ('simulation', 'crossfloor'):
        env = rendering_environment(gazebo_rendering)
    elif name == 'rviz':
        env = rendering_environment(rviz_rendering)
    with path.open('w') as output:
        process = subprocess.Popen(
            ['ros2', 'launch', 'legbot_bringup', launches[name], *arguments],
            stdout=output, stderr=subprocess.STDOUT, start_new_session=True, env=env)
    processes.append((name, process))
    print(f'启动 {name}，日志：{path.resolve()}')

log_offsets = {}

def check_processes():
    for name, process in processes:
        # ros2 launch 内一个子节点崩溃时，launch 本身可能仍活着。
        # 增量检查其标准退出报告；不能只检查父进程 PID。
        with (log_dir / f'{name}.log').open(errors='replace') as log:
            log.seek(log_offsets.get(name, 0))
            while True:
                position = log.tell()
                line = log.readline()
                if not line or not line.endswith('\n'):
                    log_offsets[name] = position
                    break
                if 'process has died' in line or 'Caught exception in launch' in line:
                    raise RuntimeError(f'{name} 子节点异常：{line.strip()}')
        if process.poll() is not None:
            raise RuntimeError(f'{name} 已退出（{process.returncode}），请查看 {log_dir / (name + ".log")}')

def send_group(process, sig):
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass

def stop(process):
    # 先让 ROS launch 正常关闭节点；超时才升级信号。
    send_group(process, signal.SIGINT)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        send_group(process, signal.SIGTERM)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            send_group(process, signal.SIGKILL)
            process.wait()
    # 即使 launch 主进程提前退出，也清理该组中残留的子进程。
    send_group(process, signal.SIGKILL)

def interrupted(signum, frame):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, interrupted)
try:
    # flock 无法发现上次退出后脱离进程组的旧 Gazebo。
    # 必须先检查 DDS 域，避免把旧控制器的 active 状态误当作本次启动成功。
    rclpy.init()
    node = rclpy.create_node('legbot_simulation_startup')
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    if client.wait_for_service(timeout_sec=2.0):
        raise RuntimeError(
            f'DDS 域 {domain} 已有 controller_manager，可能存在旧仿真。'
            '请退出旧实例，或设置 LEGBOT_SIM_DOMAIN_ID 为另一个空闲域后重试；未启动新仿真。')
    if cross_floor:
        start('crossfloor', f'gui:={str(gui).lower()}', 'rviz:=false',
              'navigation_source:=ground_truth', f'interactive:={str(interactive_3d).lower()}',
              f'goal_x:={goal[0]}', f'goal_y:={goal[1]}', f'goal_z:={goal[2]}')
        if rviz:
            start('rviz', 'use_sim_time:=true',
                  f'config:={share / "rviz/scan_crossfloor.rviz"}')
        print('RViz 操作：选择 3D Nav Goal，在楼层点云按下鼠标并拖动方向，松开即规划执行。' if interactive_3d else '自动目标模式。')
        if not interactive_3d:
            print(f'跨楼层模式：building_pct 目标 {goal}；Gazebo 世界目标 {(goal[0]+13, goal[1], goal[2])}。')
        print('等待 RViz 3D Nav Goal 目标。按 Ctrl+C 停止。' if interactive_3d else '等待 PCT 规划和起身完成。按 Ctrl+C 停止。')
        print(f'路径与执行状态：tail -f {log_dir}/crossfloor.log')
        announced = False
        while True:
            check_processes()
            if not announced and 'PCT cross-floor mission started:' in (log_dir / 'crossfloor.log').read_text(errors='replace'):
                print('PCT 三维参考路径已接入 SCAN，开始跨楼层任务。')
                announced = True
            time.sleep(1)

    # 第一步只启动 Gazebo 和机器人。默认 FAST-LIO 导航，保留激光雷达，禁用真值替代定位。
    start('simulation', f'gui:={str(gui).lower()}', 'headless_rendering:=true',
          'enable_lidar:=true', 'navigation_source:=fastlio', 'publish_ground_truth:=false')
    required = {'joint_state_broadcaster', 'imu_sensor_broadcaster', 'rl_quadruped_controller'}
    deadline = time.monotonic() + timeout
    pending = None
    while True:
        check_processes()
        if time.monotonic() >= deadline:
            raise RuntimeError('控制器未在规定时间内激活，请检查 simulation.log')
        rclpy.spin_once(node, timeout_sec=0.2)
        if pending is not None and pending.done():
            active = {item.name for item in pending.result().controller if item.state == 'active'}
            pending = None
            if required <= active:
                break
        if pending is None and client.service_is_ready():
            pending = client.call_async(ListControllers.Request())
    node.destroy_node()
    node = None
    rclpy.shutdown()

    # 控制器就绪后再启动站立状态机。它会等待定位数据和规划结果，再允许行走。
    # FAST-LIO 必须显式使用仿真配置（默认 launch 配置针对真机）。
    start('control', 'use_sim_time:=true')
    start('fastlio', 'use_sim_time:=true', f'config:={config}')
    start('scan', 'use_sim_time:=true')
    start('waypoints', 'use_sim_time:=true')
    if rviz:
        start('rviz', 'use_sim_time:=true')
    print(f'所有入口已启动，等待 FAST-LIO 初始化。DDS 域：{domain}；按 Ctrl+C 退出。')
    print(f'实时查看定位/任务状态：tail -f {log_dir}/fastlio.log {log_dir}/control.log')
    print(f'额外调试终端须设置：export ROS_DOMAIN_ID={domain} ROS_LOCALHOST_ONLY=1')
    announced = False
    while True:
        check_processes()
        if not announced and 'FAST-LIO ready: enabling SCAN waypoint mission' in (log_dir / 'control.log').read_text(errors='replace'):
            print('导航数据已就绪：可在 RViz 使用 2D Goal Pose 设置平地目标。')
            announced = True
        time.sleep(1)
except KeyboardInterrupt:
    print('\n正在停止本次仿真导航……')
except Exception as error:
    print(f'启动/运行失败：{error}', file=sys.stderr)
    sys.exit(1)
finally:
    # 清理期间忽略重复 Ctrl+C，避免导航节点或 Gazebo 留在后台。
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    # 先向所有导航进程发停止信号，最后单独停止第一个启动的 Gazebo 入口。
    for _, process in reversed(processes[1:]):
        send_group(process, signal.SIGINT)
    for _, process in reversed(processes[1:]):
        stop(process)
    if processes:
        stop(processes[0][1])
    detached = cleanup_partition(os.environ['IGN_PARTITION'])
    if detached:
        print(f'已清理脱离进程组的本次仿真进程：{detached}')
    print(f'本次启动的进程已清理。日志保留在：{log_dir.resolve()}')
PY
