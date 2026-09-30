#!/usr/bin/env python3
"""Prepare and supervise saved-PCD navigation in the existing Gazebo world."""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from simulation_cleanup import cleanup_partition
from navigation_core import prepare_map
from mapping import rendering_environment, stop_process


def positive(value):
    v=float(value)
    if not math.isfinite(v) or v<=0:raise argparse.ArgumentTypeError('必须是有限正数')
    return v


def main():
    parser=argparse.ArgumentParser(description='加载已有 PCD，生成 PCT 地图，RViz 初始定位配准后导航（仿真）。')
    parser.add_argument('pcd',nargs='?',type=Path,help='PCD 路径；省略时选择 maps/ 下最近保存的 map.pcd')
    parser.add_argument('--prepare-only',action='store_true',help='仅生成地图缓存，不启动仿真')
    parser.add_argument('--check',action='store_true',help='检查环境和 PCD 文件，不生成地图或启动仿真')
    parser.add_argument('--resolution',type=positive,default=0.15,help='规划栅格分辨率，默认 0.15 米')
    parser.add_argument('--slice-height',type=positive,default=0.5,help='楼层采样间隔，默认 0.5 米')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--no-gazebo-gui',action='store_true')
    parser.add_argument('--no-rviz',action='store_true')
    parser.add_argument('--software-rendering',action='store_true')
    parser.add_argument('--gpu-rendering',action='store_true')
    parser.add_argument('--world',type=Path,help='与 PCD 对应的 Gazebo world；默认 Building.sdf')
    parser.add_argument('--duration',type=positive,help='测试用：启动控制器后指定秒数自动退出')
    args=parser.parse_args()
    if args.software_rendering and args.gpu_rendering:parser.error('请选择一种渲染方式')
    if args.pcd is None:
        maps=list((ROOT/'maps').glob('*/map.pcd'))
        if not maps:parser.error('没有找到地图，请指定已有 PCD 路径')
        args.pcd=max(maps,key=lambda p:p.stat().st_mtime_ns)
    pcd=args.pcd.expanduser().resolve()
    if not pcd.is_file():parser.error('PCD 文件不存在：'+str(pcd))
    if args.world and not args.world.is_file():parser.error('world 文件不存在')
    print('PCD：',pcd,flush=True)
    if args.check:
        import rclpy, yaml
        from navigation_core import read_pcd
        from ament_index_python.packages import get_package_share_directory
        for name in ('legbot_bringup','pct_planner','legbot_rviz_tools'):
            get_package_share_directory(name)
        folder=Path(get_package_share_directory('pct_planner'))/'planner'
        sys.path[:0]=[str(folder),str(folder/'scripts')]
        from planner_wrapper import TomogramPlanner
        from config import Config
        TomogramPlanner(Config())
        points=read_pcd(pcd)
        print('PASS：PCD 有效，%d 点；ROS / PCT / RViz 工具可用。'%len(points))
        return 0
    print('准备分层可通行地图（CPU；相同 PCD 和参数会复用缓存）……',flush=True)
    cache=prepare_map(pcd,ROOT/'data/navigation',args.resolution,args.slice_height)
    info=json.loads((cache/'map.json').read_text())
    print('地图缓存：%s；可通行格：%d'%(cache,info['walkable_cells']),flush=True)
    if args.prepare_only:return 0
    import yaml
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from ament_index_python.packages import get_package_share_directory
    from controller_manager_msgs.srv import ListControllers
    share=Path(get_package_share_directory('legbot_bringup'))
    config=yaml.safe_load((share/'rviz/scan_crossfloor.rviz').read_text())
    vm=config['Visualization Manager'];vm['Global Options']['Fixed Frame']='navigation_map'
    def convert(items):
        result=[]
        for item in items:
            if item.get('Class')=='rviz_default_plugins/InteractiveMarkers':continue
            if item.get('Class')=='rviz_common/Group':
                item['Displays']=convert(item.get('Displays',[]))
            topic=item.get('Topic')
            value=topic.get('Value') if isinstance(topic,dict) else topic
            replacements={'/global_points':'/navigation/map','/tomogram':'/navigation/traversability'}
            if value in replacements:
                item['Topic']={'Value':replacements[value],'Reliability Policy':'Reliable',
                    'Durability Policy':'Transient Local','History Policy':'Keep Last','Depth':1}
                item['Name']='Loaded PCD' if value=='/global_points' else 'Walkable surfaces'
                item['Color Transformer']='AxisColor'
            result.append(item)
        return result
    vm['Displays']=convert(vm['Displays'])
    vm['Displays'].append({'Class':'rviz_default_plugins/Marker','Name':'Navigation status','Enabled':True,
        'Value':True,'Topic':{'Value':'/navigation/status','Reliability Policy':'Reliable',
        'Durability Policy':'Transient Local','History Policy':'Keep Last','Depth':1}})
    for t in vm['Tools']:
        if t['Class']=='legbot_rviz_tools/PoseEstimate3D':t['Height offset']=0.5
    view=vm['Views']['Current']
    center=(__import__('numpy').array(info['bounds'][0])+info['bounds'][1])/2
    extent=max(info['bounds'][1][i]-info['bounds'][0][i] for i in range(3))
    view.update({'Target Frame':'navigation_map','Focal Point':dict(zip(('X','Y','Z'),map(float,center))),
                 'Distance':max(12.0,extent*1.6)})
    vm['Views']['Saved']=None
    rviz_config=cache/'navigation.rviz';rviz_config.write_text(yaml.safe_dump(config,sort_keys=False))
    stamp=datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+str(os.getpid())
    log=ROOT/'log/navigation'/stamp;log.mkdir(parents=True)
    processes=[];offsets={};node=None
    mode='software' if args.software_rendering else 'gpu'
    if not(args.software_rendering or args.gpu_rendering) and shutil.which('nvidia-smi'):
        try:
            probe=subprocess.run(['nvidia-smi','-L'],capture_output=True,text=True,timeout=5)
            if probe.returncode and 'Driver/library version mismatch' in probe.stdout+probe.stderr:
                mode='software'
                print('NVIDIA 驱动与库版本不一致，自动使用 Mesa 软件渲染（速度较慢）。',flush=True)
        except subprocess.TimeoutExpired:
            pass
    def render_env():
        env=rendering_environment(mode)
        if mode=='software':
            env['__GLX_VENDOR_LIBRARY_NAME']='mesa'
            mesa=Path('/usr/share/glvnd/egl_vendor.d/50_mesa.json')
            if mesa.is_file():env['__EGL_VENDOR_LIBRARY_FILENAMES']=str(mesa)
        return env
    # A desktop session can use GLX; do not force the EGL device path.
    headless_rendering=not bool(os.environ.get('DISPLAY'))
    print('渲染：%s；后端：%s'%(mode,'EGL' if headless_rendering else 'GLX'),flush=True)
    def launch(name,command,render=False):
        with (log/(name+'.log')).open('w') as f:
            child=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,cwd=ROOT,
                env=render_env() if render else None)
        processes.append((name,child));print('启动 '+name+'，日志：'+str(log/(name+'.log')),flush=True)
    def start(name,file,*extra,render=False):
        launch(name,['ros2','launch','legbot_bringup',file,*extra],render)
    def check():
        for name,process in processes:
            if process.poll() is not None:raise RuntimeError(name+' exited; see '+str(log))
            with (log/(name+'.log')).open(errors='replace') as f:
                f.seek(offsets.get(name,0))
                while True:
                    pos=f.tell();line=f.readline()
                    if not line or not line.endswith('\n'):
                        offsets[name]=pos;break
                    if ('process has died' in line or 'Caught exception in launch' in line or
                        ('Request to create entity' in line and 'timed out' in line)):
                        raise RuntimeError(line.strip())
                    if name=='navigation' and any(x in line for x in ('ALIGN','GOAL','ARRIVED')):
                        print(line.strip(),flush=True)
    def interrupt(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    result=0
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node=rclpy.create_node('navigation_startup')
        client=node.create_client(ListControllers,'/controller_manager/list_controllers')
        if client.wait_for_service(timeout_sec=2):
            raise RuntimeError('DDS 域已有 controller_manager，请先退出旧仿真')
        world=['world:='+str(args.world.resolve())] if args.world else []
        start('simulation','simulation.launch.py','navigation_source:=ground_truth','publish_ground_truth:=true',
              'enable_lidar:=true','headless_rendering:='+str(headless_rendering).lower(),
              'gui:=false',*world,render=True)
        end=time.monotonic()+positive(os.environ.get('LEGBOT_SIM_START_TIMEOUT','120'));future=None
        while True:
            check()
            if time.monotonic()>end:raise RuntimeError('控制器启动超时')
            rclpy.spin_once(node,timeout_sec=.1)
            if future and future.done():
                active={x.name for x in future.result().controller if x.state=='active'}
                if {'joint_state_broadcaster','imu_sensor_broadcaster','rl_quadruped_controller'}<=active:break
                future=None
            if future is None and client.service_is_ready():
                startup_log=(log/'simulation.log').read_text(errors='replace')
                if any('Configured and activated' in line and 'rl_quadruped_controller' in line for line in startup_log.splitlines()):
                    future=client.call_async(ListControllers.Request())
        # Start the physics server independently of the GUI. Wait for a real
        # controller service before connecting the viewer to this partition.
        if not(args.headless or args.no_gazebo_gui):
            launch('gazebo_gui',['ign','gazebo','-g'],render=True)
        start('control','go2_demo_control.launch.py','use_sim_time:=true','odom_topic:=/Odometry_gazebo',
              'cloud_topic:=/livox/points_world','localization_label:=PCD navigation')
        start('scan','scan.launch.py','use_sim_time:=true','navi_mode:=3','global_path_topic:=/pct_path',
              'frame_id:=odom','manual_goal_use_message_z:=true','odom_topic:=/Odometry_gazebo',
              'cloud_topic:=/livox/points_world','sensor_pose_topic:=/go2/lidar_pose','finish_dist_z:=0.6')
        launch('navigation',[sys.executable,str(ROOT/'tools/navigation_node.py'),str(cache),
                              '--ros-args','-p','use_sim_time:=true'])
        if not(args.headless or args.no_rviz):
            start('rviz','scan_rviz.launch.py','use_sim_time:=true','config:='+str(rviz_config),render=True)
        print('先在 PCD 上用 3D Pose Estimate 标记机械狗当前位置并拖动朝向；ALIGNED 后用 3D Nav Goal。',flush=True)
        print('PCD 必须对应当前仿真世界。没有成功配准前，不发送行走任务。Ctrl+C 退出。',flush=True)
        end=time.monotonic()+args.duration if args.duration else math.inf
        while time.monotonic()<end:
            check();time.sleep(.3)
    except KeyboardInterrupt:print('正在停止本次地图导航……')
    except Exception as error:print('导航失败：'+str(error),file=sys.stderr);result=1
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        for _,process in reversed(processes):stop_process(process)
        cleanup_partition(os.environ['IGN_PARTITION'])
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        print('本次进程已清理。日志：'+str(log))
    return result


if __name__=='__main__':
    try:sys.exit(main())
    except (ValueError,OSError,KeyError) as error:sys.exit(str(error))
