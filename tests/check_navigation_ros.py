#!/usr/bin/env python3
"""Isolated ROS test: no simulator, no motor controller, synthetic saved map and sensors."""
from pathlib import Path
import os
import signal
import subprocess
import sys
import tempfile
import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path as RosPath
from std_msgs.msg import Bool,Header,String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'));sys.path.insert(0,str(ROOT/'tests'))
from check_navigation_offline import plane,write_pcd
from navigation_core import prepare_map
rclpy.init()
node=rclpy.create_node('navigation_integration_test')
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
paths=[];localized=[];states=[];maps=[]
node.create_subscription(RosPath,'/pct_path',lambda m:paths.append(m),latched)
node.create_subscription(Bool,'/navigation/localized',lambda m:localized.append(m.data),latched)
node.create_subscription(String,'/navigation/state',lambda m:states.append(m.data),latched)
node.create_subscription(PointCloud2,'/navigation/map',lambda m:maps.append(m.width),latched)
odom=node.create_publisher(Odometry,'/Odometry_gazebo',qos_profile_sensor_data)
cloud=node.create_publisher(PointCloud2,'/livox/points_world',qos_profile_sensor_data)
ready=node.create_publisher(Bool,'/go2/demo_ready',latched)
initial=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
goal=node.create_publisher(PoseStamped,'/pct/nav_goal',10)
ack=node.create_publisher(PoseStamped,'/scan/goal_stop_ack',10)
process=None
with tempfile.TemporaryDirectory(prefix='legbot-nav-test-') as d:
    directory=Path(d)
    points=plane()
    # A wall makes orientation/translation alignment geometrically constrained.
    y,z=np.meshgrid(np.arange(-3,3.01,.08),np.arange(.1,1.8,.08))
    wall=np.column_stack((np.full(y.size,-3.),y.ravel(),z.ravel()))
    points=np.vstack((points,wall))
    pcd=directory/'test.pcd';write_pcd(pcd,points)
    cache=prepare_map(pcd,directory/'cache')
    msg=point_cloud2.create_cloud_xyz32(Header(frame_id='odom'),(points+[-27,6,0]).astype(np.float32))
    logdir=ROOT/'log/navigation_test';logdir.mkdir(parents=True,exist_ok=True)
    log=(logdir/'node.log').open('w')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            assert process.poll() is None,'Node exited; see '+str(logdir)
            m=Odometry();m.header.frame_id='odom';m.pose.pose.orientation.w=1.0
            m.pose.pose.position.x=-27.;m.pose.pose.position.y=6.;m.pose.pose.position.z=.5
            odom.publish(m);cloud.publish(msg);ready.publish(Bool(data=True))
            rclpy.spin_once(node,timeout_sec=.1)
    def set_pose(x):
        m=PoseWithCovarianceStamped();m.header.frame_id='navigation_map'
        m.pose.pose.position.x=x;m.pose.pose.position.z=.5;m.pose.pose.orientation.w=1.
        initial.publish(m)
    try:
        process=subprocess.Popen([sys.executable,str(ROOT/'tools/navigation_node.py'),str(cache)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        spin(4)
        g=PoseStamped();g.header.frame_id='navigation_map';g.pose.position.x=1.;g.pose.position.y=1.;g.pose.orientation.w=1.
        goal.publish(g);spin(1)
        assert not paths and any('first set' in s for s in states),'Goal was not gated before localization'
        set_pose(100.);spin(2)
        assert True not in localized,'Bad alignment was accepted'
        set_pose(0.);spin(4)
        assert localized[-1] is True,states[-3:]
        assert maps and maps[-1]>1000,'PCD was not published'
        goal.publish(g);spin(3)
        assert len(paths)==1 and paths[0].header.frame_id=='odom',states[-4:]
        end=paths[0].poses[-1]
        assert abs(end.pose.position.x+26)<.3 and abs(end.pose.position.y-7)<.3,'Goal frame conversion failed'
        assert abs(end.pose.position.z-.5)<.1
        set_pose(0.);spin(1)
        assert 'ALIGNMENT REJECTED' in states[-1]
        ack.publish(end);spin(1)
        assert any('ARRIVED' in s for s in states)
        print('PASS saved-PCD publication, alignment rejection/acceptance, goal gating, native PCT route, SCAN frame conversion and completion')
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=6)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close()
node.destroy_node();rclpy.shutdown()
