#!/usr/bin/env python3
"""Manual simulation mapping supervisor. No real hardware launch is used."""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import select
import signal
import shutil
import subprocess
import sys
import termios
import time
import tty
from simulation_cleanup import cleanup_partition

ROOT = Path(__file__).resolve().parents[1]
KEYS = {'w': (1, 0, 0), 's': (-1, 0, 0), 'a': (0, 1, 0),
        'd': (0, -1, 0), 'q': (0, 0, 1), 'e': (0, 0, -1)}


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('必须为有限正数')
    return result


def arguments(argv=None):
    parser = argparse.ArgumentParser(description='GO2 仿真键盘扫图，保存 FAST-LIO 三维 PCD 地图。',
        epilog='W/S 前后，A/D 横移，Q/E 转向，空格/X 停止，P 保存，Esc 或 Ctrl+C 保存退出。'
               '松开移动键后 0.5 秒内自动归零。默认 maps/<时间戳-PID>/map.pcd。')
    parser.add_argument('--headless', action='store_true', help='关闭 Gazebo GUI 和 RViz')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--no-gazebo-gui', action='store_true')
    parser.add_argument('--check', action='store_true', help='仅检查环境，不启动仿真')
    parser.add_argument('--output', type=Path, help='地图输出目录，必须为尚不存在的新目录')
    parser.add_argument('--speed', type=positive, default=0.25, help='前后速度 m/s，默认 0.25，最大 0.75')
    parser.add_argument('--turn-speed', type=positive, default=0.4, help='转向 rad/s，默认 0.4，最大 1.0')
    parser.add_argument('--duration', type=positive, help='就绪后运行指定秒数并保存退出；可用于无终端静止采样')
    parser.add_argument('--gpu-rendering', action='store_true')
    parser.add_argument('--software-rendering', action='store_true')
    parser.add_argument('--gazebo-rendering', choices=['gpu', 'software'])
    parser.add_argument('--rviz-rendering', choices=['gpu', 'software'])
    args = parser.parse_args(argv)
    if args.speed > 0.75 or args.turn_speed > 1.0:
        parser.error('速度上限：--speed 0.75，--turn-speed 1.0')
    if args.gpu_rendering and args.software_rendering:
        parser.error('GPU 与软件渲染参数不能同时指定')
    return args


class KeyboardCommand:
    def __init__(self, speed, turn_speed):
        self.limits = (speed, min(speed, 0.25), turn_speed)
        self.velocity = (0., 0., 0.)
        self.deadline = 0.

    def stop(self):
        self.velocity = (0., 0., 0.)
        self.deadline = 0.

    def press(self, key, now):
        self.stop()
        if key.lower() in KEYS:
            self.velocity = tuple(a*b for a, b in zip(KEYS[key.lower()], self.limits))
            self.deadline = now + 0.5

    def current(self, now):
        return self.velocity if now < self.deadline else (0., 0., 0.)


def verify_pcd(path):
    """Check a complete binary PCD before promoting it to the final filename."""
    fields = {}
    with path.open('rb') as stream:
        for _ in range(64):
            line = stream.readline(4096).decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            name, *values = line.split()
            fields[name] = values
            if name == 'DATA':
                break
        else:
            raise RuntimeError('PCD 文件缺少 DATA 头')
        points = int(fields['POINTS'][0])
        stride = sum(int(size)*int(count) for size, count in
                     zip(fields['SIZE'], fields['COUNT']))
        if fields['DATA'] != ['binary'] or points <= 0 or stride <= 0:
            raise RuntimeError('地图为空或 PCD 格式不正确')
        if path.stat().st_size - stream.tell() != points * stride:
            raise RuntimeError('PCD 数据长度不完整')
    return points


def rendering_environment(mode):
    env = os.environ.copy()
    env.pop('LIBGL_ALWAYS_SOFTWARE', None)
    env.pop('GALLIUM_DRIVER', None)
    if mode == 'software':
        env.update(LIBGL_ALWAYS_SOFTWARE='1', GALLIUM_DRIVER='llvmpipe',
                   __GLX_VENDOR_LIBRARY_NAME='mesa')
        mesa=Path('/usr/share/glvnd/egl_vendor.d/50_mesa.json')
        if mesa.is_file():
            env['__EGL_VENDOR_LIBRARY_FILENAMES']=str(mesa)
    return env


def send_group(process, sig):
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass


def stop_process(process):
    for sig, timeout in [(signal.SIGINT, 12), (signal.SIGTERM, 3), (signal.SIGKILL, 3)]:
        send_group(process, sig)
        try:
            process.wait(timeout=timeout)
            break
        except subprocess.TimeoutExpired:
            continue
    send_group(process, signal.SIGKILL)


def choose_rendering(label, selected, default, check):
    if selected:
        return selected
    if check or not sys.stdin.isatty():
        return default
    answer = input(f'{label} 使用 GPU 渲染？[Y/n]：').strip().lower()
    if answer not in ('', 'y', 'yes', 'n', 'no'):
        raise ValueError('请输入 y 或 n')
    return 'software' if answer in ('n', 'no') else 'gpu'


def main(argv=None):
    args = arguments(argv)
    # Delay ROS imports so --help and the pure regression tests work without ROS.
    import yaml
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_share_directory
    from controller_manager_msgs.srv import ListControllers
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Int32
    from std_srvs.srv import Trigger

    domain = int(os.environ.get('ROS_DOMAIN_ID', '178'))
    timeout = positive(os.environ.get('LEGBOT_SIM_START_TIMEOUT', '120'))
    if not 0 <= domain <= 232:
        raise ValueError('DDS 域必须在 0～232 之间')
    share = Path(get_package_share_directory('legbot_bringup'))
    for relative in ['launch/simulation.launch.py', 'launch/fastlio.launch.py',
                     'launch/scan_rviz.launch.py', 'config/fastlio_sim.yaml']:
        if not (share / relative).is_file():
            raise RuntimeError(f'缺少 {share / relative}，请先执行 ./build.sh')
    if not (ROOT / 'tools/mapping.rviz').is_file():
        raise RuntimeError('缺少 tools/mapping.rviz')
    if args.output and args.output.expanduser().exists():
        raise ValueError('--output 目录已存在，请指定新目录，避免覆盖已有地图')
    if args.check:
        print('PASS 建图环境、ROS 消息、服务和启动文件检查；未启动仿真。')
        return 0
    if not sys.stdin.isatty() and args.duration is None:
        raise ValueError('键盘扫图需要交互终端；SSH 请使用 ssh -t。无交互静止采样可指定 --duration 秒数。')
    common_mode = 'software' if args.software_rendering else ('gpu' if args.gpu_rendering else None)
    gazebo_mode = choose_rendering('Gazebo', args.gazebo_rendering or common_mode, 'gpu', False)
    rviz = not (args.headless or args.no_rviz)
    rviz_mode = choose_rendering('RViz', args.rviz_rendering or common_mode, 'gpu', False) if rviz else 'gpu'

    if (gazebo_mode=='gpu' or rviz_mode=='gpu') and shutil.which('nvidia-smi'):
        try:
            probe=subprocess.run(['nvidia-smi','-L'],capture_output=True,text=True,timeout=5)
            if probe.returncode and 'Driver/library version mismatch' in probe.stdout+probe.stderr:
                gazebo_mode=rviz_mode='software'
                print('NVIDIA 驱动与库版本冲突，本次建图自动切换 Mesa 软件渲染。')
        except subprocess.TimeoutExpired:
            pass

    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S') + f'-{os.getpid()}'
    output = (args.output.expanduser() if args.output else ROOT / 'maps' / stamp).resolve()
    output.mkdir(parents=True, exist_ok=False)
    log_dir = ROOT / 'log/mapping' / stamp
    log_dir.mkdir(parents=True)
    pending_map = output / 'map.pending.pcd'
    config = yaml.safe_load((share / 'config/fastlio_sim.yaml').read_text())
    params = config['/**']['ros__parameters']
    params['pcd_save']['pcd_save_en'] = True
    # This FAST-LIO fork accumulates its saved cloud inside publish_map().
    params['publish']['map_en'] = True
    params['map_file_path'] = str(pending_map)
    config_path = output / 'fastlio.yaml'
    config_path.write_text(yaml.safe_dump(config, allow_unicode=True))
    print(f'地图目录：{output}\n日志目录：{log_dir}\nDDS 域：{domain}')

    processes, offsets = [], {}
    node = None
    terminal = None
    fastlio_started = False
    ready = False
    exit_code = 0
    saved = False
    command = KeyboardCommand(args.speed, args.turn_speed)
    observed = {'odom': -math.inf, 'cloud': -math.inf, 'frame': '', 'points': 0}

    def start(name, launch, *extra, mode=None):
        with (log_dir / f'{name}.log').open('w') as stream:
            process = subprocess.Popen(['ros2', 'launch', 'legbot_bringup', launch, *extra],
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
                env=rendering_environment(mode) if mode else None, cwd=ROOT)
        processes.append((name, process))
        print(f'启动 {name}：{log_dir / (name + ".log")}')

    def check_processes():
        for name, process in processes:
            with (log_dir / f'{name}.log').open(errors='replace') as stream:
                stream.seek(offsets.get(name, 0))
                while True:
                    position = stream.tell()
                    line = stream.readline()
                    if not line or not line.endswith('\n'):
                        offsets[name] = position
                        break
                    if 'process has died' in line or 'Caught exception in launch' in line:
                        raise RuntimeError(f'{name} 子节点异常：{line.strip()}')
            if process.poll() is not None:
                raise RuntimeError(f'{name} 已退出，请检查日志')

    def publish_command():
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = command.current(time.monotonic())
        velocity_pub.publish(msg)

    def spin_for(seconds, mode=None, check=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if check:
                check_processes()
            publish_command()
            if mode is not None:
                mode_pub.publish(Int32(data=mode))
            rclpy.spin_once(node, timeout_sec=0.05)

    def on_odom(msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if msg.header.frame_id and all(math.isfinite(v) for v in values) and 0.5 <= math.sqrt(sum(v*v for v in values[3:])) <= 1.5:
            observed['odom'] = time.monotonic()

    def on_cloud(msg):
        if msg.header.frame_id and msg.width * msg.height > 0 and msg.data:
            observed['cloud'] = time.monotonic()

    def on_map(msg):
        observed['frame'] = msg.header.frame_id
        observed['points'] = msg.width * msg.height

    def save_map():
        nonlocal saved
        command.stop()
        publish_command()
        if not observed['points']:
            raise RuntimeError('尚未收到非空地图，未保存空文件')
        if not save_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('/map_save 服务不可用')
        # Do not mistake a partial file from an earlier failed request for success.
        before = pending_map.stat().st_mtime_ns if pending_map.exists() else None
        future = save_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 60.0
        while not future.done() and time.monotonic() < deadline:
            publish_command()
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError('保存超时；临时文件保留在地图目录')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        if not pending_map.exists() or pending_map.stat().st_mtime_ns == before:
            raise RuntimeError('保存服务没有生成新地图文件')
        points = verify_pcd(pending_map)
        pending_map.replace(output / 'map.pcd')
        metadata = {'format': 'PCD binary', 'frame_id': observed['frame'],
                    'points': points, 'saved_at': datetime.datetime.now().isoformat(),
                    'source': 'FAST-LIO simulation', 'ros_domain_id': domain}
        (output / 'map.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + '\n')
        saved = True
        print(f'地图已保存：{output / "map.pcd"}（{points} 点，坐标系 {observed["frame"]}）')

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        # Keep the ROS context alive on Ctrl+C so saving can finish first.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('legbot_mapping_keyboard')
        velocity_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        mode_pub = node.create_publisher(Int32, '/go2/control_mode', 10)
        node.create_subscription(Odometry, '/fast_lio/odometry_base', on_odom, qos_profile_sensor_data)
        node.create_subscription(PointCloud2, '/fast_lio/cloud_registered', on_cloud, qos_profile_sensor_data)
        node.create_subscription(PointCloud2, '/fast_lio/map', on_map, qos_profile_sensor_data)
        save_client = node.create_client(Trigger, '/map_save')
        controller = node.create_client(ListControllers, '/controller_manager/list_controllers')
        if controller.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'DDS 域 {domain} 已有控制器，请先退出旧仿真')
        start('simulation', 'simulation.launch.py',
              f'gui:={str(not (args.headless or args.no_gazebo_gui)).lower()}',
              'headless_rendering:='+str(not bool(os.environ.get('DISPLAY'))).lower(), 'enable_lidar:=true',
              'navigation_source:=fastlio', 'publish_ground_truth:=false', mode=gazebo_mode)
        deadline, request = time.monotonic() + timeout, None
        required = {'joint_state_broadcaster', 'imu_sensor_broadcaster', 'rl_quadruped_controller'}
        while True:
            spin_for(0.1)
            if time.monotonic() > deadline:
                raise RuntimeError('控制器启动超时')
            if request is not None and request.done():
                active = {item.name for item in request.result().controller if item.state == 'active'}
                if required <= active:
                    break
                request = None
            if request is None and controller.service_is_ready():
                startup_log=(log_dir/'simulation.log').read_text(errors='replace')
                if any('Configured and activated' in line and 'rl_quadruped_controller' in line for line in startup_log.splitlines()):
                    request = controller.call_async(ListControllers.Request())
        start('fastlio', 'fastlio.launch.py', 'use_sim_time:=true', f'config:={config_path}')
        fastlio_started = True
        if rviz:
            start('rviz', 'scan_rviz.launch.py', 'use_sim_time:=true',
                  f'config:={ROOT / "tools/mapping.rviz"}', mode=rviz_mode)
        print('控制器就绪，正在起身并等待 FAST-LIO……')
        spin_for(1.0, mode=2)
        spin_for(3.0)
        spin_for(1.0, mode=2)
        spin_for(5.0)
        deadline = time.monotonic() + timeout
        while not (time.monotonic() - observed['odom'] < 2.0 and
                   time.monotonic() - observed['cloud'] < 2.0 and observed['points'] > 0):
            spin_for(0.1)
            if time.monotonic() > deadline:
                raise RuntimeError('FAST-LIO 定位或地图就绪超时')
        spin_for(1.0, mode=3)
        ready = True
        print('建图就绪：W/S 前后，A/D 横移，Q/E 转向，空格/X 停止，P 保存，Esc/Ctrl+C 保存退出。')
        print('请让终端保持焦点；持续按移动键，松开后 0.5 秒内归零。')
        if sys.stdin.isatty():
            terminal = termios.tcgetattr(sys.stdin.fileno())
            termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
            tty.setcbreak(sys.stdin.fileno())
        deadline = time.monotonic() + args.duration if args.duration else math.inf
        while time.monotonic() < deadline:
            check_processes()
            if terminal is not None and select.select([sys.stdin], [], [], 0)[0]:
                key = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
                if not key or key == '\x1b':
                    break
                if key.lower() == 'p':
                    command.stop()
                    try:
                        save_map()
                    except Exception as error:
                        print(f'保存失败：{error}；可按 P 重试。', file=sys.stderr)
                    # Drop any movement keys queued while the service was saving.
                    termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
                else:
                    command.press(key, time.monotonic())
            publish_command()
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print('\n停止移动，正在保存并退出……')
    except Exception as error:
        print(f'建图失败：{error}', file=sys.stderr)
        exit_code = 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if terminal is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, terminal)
        if node is not None and rclpy.ok() and processes:
            command.stop()
            try:
                spin_for(1.5 if ready else 0.2, check=False)
                if ready:
                    spin_for(0.8, mode=2, check=False)
                if fastlio_started:
                    save_map()
            except Exception as error:
                print(f'退出保存未完成：{error}' + ('；上次成功保存的 map.pcd 仍保留。' if saved else ''), file=sys.stderr)
                exit_code = 1
        for _, process in reversed(processes):
            stop_process(process)
        try:
            detached = cleanup_partition(os.environ['IGN_PARTITION'])
            if detached:
                print(f'已清理脱离进程组的本次仿真进程：{detached}')
        except Exception as error:
            print(f'残留进程清理失败：{error}', file=sys.stderr)
            exit_code = 1
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f'本次建图进程已清理。日志：{log_dir}')
    return exit_code


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, RuntimeError) as error:
        sys.exit(str(error))
