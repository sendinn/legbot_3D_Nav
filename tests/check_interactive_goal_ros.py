#!/usr/bin/env python3
"""ROS integration test with synthetic odometry; runs no simulator or robot."""
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate

rclpy.init()
node = rclpy.create_node('test_interactive_goal')
robot_coords = (7.5, 6.0, 0.5)
paths = []
updates = []
node.create_subscription(RosPath, '/pct_path', lambda m: paths.append(m),
                         QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(InteractiveMarkerUpdate, '/basic_controls/update',
                         lambda m: updates.extend(m.markers), 10)
odom = node.create_publisher(Odometry, '/Odometry_gazebo', qos_profile_sensor_data)
ready = node.create_publisher(Bool, '/go2/demo_ready',
                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
feedback = node.create_publisher(InteractiveMarkerFeedback, '/basic_controls/feedback', 10)
nav_goal = node.create_publisher(PoseStamped, '/pct/nav_goal', 10)
pick = node.create_publisher(PointStamped, '/pct/goal_pick', 10)
processes = []
logs = []
root = Path(__file__).resolve().parents[1]
logdir = root / 'log/interactive_goal_test'
logdir.mkdir(parents=True, exist_ok=True)

def start(name, cmd):
    log = (logdir / (name + '.log')).open('w')
    logs.append(log)
    processes.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))

def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for p in processes:
            assert p.poll() is None, 'Test child exited; see ' + str(logdir)
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = robot_coords
        msg.pose.pose.orientation.w = 1.0
        odom.publish(msg)
        ready.publish(Bool(data=True))
        rclpy.spin_once(node, timeout_sec=0.05)

def event(kind, x=2., y=-3., z=4.5):
    msg = InteractiveMarkerFeedback()
    msg.header.frame_id = 'building_pct'
    msg.client_id = 'integration_test'
    msg.marker_name = 'goal_3d'
    msg.control_name = 'menu' if kind == InteractiveMarkerFeedback.MENU_SELECT else 'move_X'
    msg.event_type = kind
    msg.menu_entry_id = 1
    msg.pose.orientation.w = 1.
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    feedback.publish(msg)

try:
    start('planner', ['ros2','launch','legbot_bringup','pct_plan.launch.py',
                      'use_sim_time:=false','interactive:=true'])
    start('mission', ['ros2','run','legbot_bringup','pct_path_mission','--ros-args',
                     '-p','use_sim_time:=false','-p','allow_retarget:=true',
                     '-p','publish_map_tf:=false','-p','odom_topic:=/Odometry_gazebo'])
    spin(5)
    assert not paths, 'Automatic route published before user execution'
    point = PointStamped()
    point.header.frame_id = 'odom'
    point.point.x, point.point.y, point.point.z = 15., -3., 4.5
    pick.publish(point)
    spin(1)
    assert updates and updates[-1].name == 'goal_3d', 'No interactive target'
    assert abs(updates[-1].pose.position.x - 2.) < 1e-6, 'Pick frame conversion failed'
    event(InteractiveMarkerFeedback.POSE_UPDATE)
    spin(1)
    assert not paths, 'Dragging unexpectedly executed a route'
    event(InteractiveMarkerFeedback.MENU_SELECT)
    spin(4)
    assert len(paths) == 1 and len(paths[-1].poses) > 2, 'First goal was not forwarded'
    assert abs(paths[-1].poses[-1].pose.position.x - 15.) < 0.75, 'World transform incorrect'
    first_stamp = paths[-1].header.stamp
    robot_coords = (8.0, 6.0, 0.5)
    event(InteractiveMarkerFeedback.POSE_UPDATE, x=1.5)
    spin(0.5)
    event(InteractiveMarkerFeedback.MENU_SELECT, x=1.5)
    spin(4)
    assert len(paths) == 2, 'Replacement goal was ignored'
    assert abs(paths[-1].poses[0].pose.position.x - 8.0) < 0.25, 'Replan did not start at current robot position'
    assert paths[-1].header.stamp != first_stamp, 'Replacement reused mission identity'
    assert abs(paths[-1].poses[-1].pose.position.x - 14.5) < 0.75
    direct = PoseStamped()
    direct.header.frame_id = 'odom'
    direct.pose.position.x, direct.pose.position.y, direct.pose.position.z = 15.0, -3.0, 4.5
    direct.pose.orientation.w = 1.0
    nav_goal.publish(direct)
    spin(4)
    assert len(paths) == 3, 'Toolbar 3D goal was not executed directly'
    assert abs(paths[-1].poses[-1].pose.position.x - 15.0) < 0.75
    event(InteractiveMarkerFeedback.POSE_UPDATE, x=-4.423065, y=-2.653502, z=1.782454)
    spin(0.5)
    event(InteractiveMarkerFeedback.MENU_SELECT)
    spin(1)
    assert len(paths) == 3, 'Blocked goal was sent'
    assert 'BLOCKED TARGET' in updates[-1].description, 'Blocked status not visible in RViz'
    event(InteractiveMarkerFeedback.POSE_UPDATE, x=999.)
    spin(0.5)
    event(InteractiveMarkerFeedback.MENU_SELECT, x=999.)
    spin(1)
    assert len(paths) == 3, 'Out-of-map goal was sent'
    print('PASS: no auto start, point selection, drag without movement, execute, retarget, invalid goal rejection')
finally:
    for p in processes:
        try:
            os.killpg(p.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    for p in processes:
        try:
            p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()
    for log in logs:
        log.close()
    node.destroy_node()
    rclpy.shutdown()
