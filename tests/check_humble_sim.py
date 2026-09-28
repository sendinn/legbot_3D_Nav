"""Bounded Fortress startup test. Starts simulation only; never publishes commands.

Run after a full build: source tools/env.sh; python3 tests/check_humble_sim.py
Uses a separate DDS domain and Ignition partition, then stops its own launch tree.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

# Set transport isolation before initializing rclpy or launching Gazebo.
os.environ['ROS_DOMAIN_ID'] = '179'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['IGN_IP'] = '127.0.0.1'
os.environ['IGN_PARTITION'] = f'legbot_humble_test_{os.getpid()}'

import rclpy
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState


def main():
    if os.environ.get('ROS_DISTRO') != 'humble':
        raise RuntimeError('Source tools/env.sh from the Humble branch first')
    log_path = Path('log/humble_sim_smoke.log').resolve()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('legbot_humble_smoke_check')
    samples = []

    def receive(message):
        if len(message.name) == 12 and len(message.position) == 12:
            if all(math.isfinite(value) for value in message.position):
                samples.append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)

    subscription = node.create_subscription(JointState, '/joint_states', receive, 10)
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    required = {'joint_state_broadcaster', 'imu_sensor_broadcaster', 'rl_quadruped_controller'}
    process = None
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([
                'ros2', 'launch', 'legbot_bringup', 'simulation.launch.py',
                'gui:=false', 'headless_rendering:=true', 'enable_lidar:=false',
                'navigation_source:=ground_truth', 'publish_ground_truth:=true',
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 120
            active = set()
            pending = None
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f'Simulation exited early; see {log_path}')
                rclpy.spin_once(node, timeout_sec=0.2)
                if pending is not None and pending.done():
                    response = pending.result()
                    active = {c.name for c in response.controller if c.state == 'active'}
                    pending = None
                if required <= active and len(samples) >= 10 and samples[-1] > samples[0]:
                    print('PASS Humble/Fortress: all three controllers active; '
                          '12 finite joint positions received over advancing simulation time')
                    print(f'Simulation log: {log_path}')
                    return
                if pending is None and client.service_is_ready():
                    pending = client.call_async(ListControllers.Request())
            raise RuntimeError(f'Simulation startup timed out; active={active}, '
                               f'joint_samples={len(samples)}; see {log_path}')
    finally:
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
