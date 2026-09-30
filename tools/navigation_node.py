#!/usr/bin/env python3
"""PCD map server, initial-pose/ICP alignment and PCT -> SCAN mission bridge."""
import json
import math
from pathlib import Path
import pickle
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
from navigation_core import nearest_surface, register_cloud, transform_points

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String
from visualization_msgs.msg import Marker
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory


def pose_matrix(pose):
    q=pose.orientation
    quat=np.array([q.x,q.y,q.z,q.w])
    if not np.isfinite(quat).all() or not 0.5<=np.linalg.norm(quat)<=1.5:
        raise ValueError('Invalid pose orientation')
    result=np.eye(4)
    result[:3,:3]=Rotation.from_quat(quat).as_matrix()
    result[:3,3]=[pose.position.x,pose.position.y,pose.position.z]
    if not np.isfinite(result).all():raise ValueError('Non-finite pose')
    return result


class MapNavigation(Node):
    def __init__(self, cache):
        super().__init__('pcd_navigation')
        self.cache=Path(cache)
        self.points=np.load(self.cache/'points.npy',allow_pickle=False)
        with (self.cache/'map.pickle').open('rb') as f:self.data=pickle.load(f)
        folder=Path(get_package_share_directory('pct_planner'))/'planner'
        sys.path[:0]=[str(folder),str(folder/'scripts')]
        from config import Config
        from planner_wrapper import TomogramPlanner
        self.planner=TomogramPlanner(Config())
        self.planner.tomo_dir=self.cache
        self.planner.loadTomogram('map')
        self.latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=self.create_publisher(PointCloud2,'/navigation/map',self.latched)
        self.free_pub=self.create_publisher(PointCloud2,'/navigation/traversability',self.latched)
        self.path_pub=self.create_publisher(RosPath,'/pct_path',self.latched)
        self.goal_pub=self.create_publisher(PoseStamped,'/scan/goal',10)
        self.complete_pub=self.create_publisher(Bool,'/scan/mission_complete',self.latched)
        self.localized_pub=self.create_publisher(Bool,'/navigation/localized',self.latched)
        self.status_pub=self.create_publisher(Marker,'/navigation/status',self.latched)
        self.state_pub=self.create_publisher(String,'/navigation/state',self.latched)
        self.tf=StaticTransformBroadcaster(self)
        self.odom=None;self.cloud=None
        self.last_odom=self.last_cloud=-math.inf
        self.map_from_odom=None
        self.ready=False;self.active=False;self.goal_stamp=None
        self.create_subscription(Odometry,'/Odometry_gazebo',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/livox/points_world',self.on_cloud,qos_profile_sensor_data)
        self.create_subscription(Bool,'/go2/demo_ready',self.on_ready,self.latched)
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.on_initial_pose,10)
        self.create_subscription(PoseStamped,'/pct/nav_goal',self.on_goal,10)
        self.create_subscription(PoseStamped,'/scan/goal_stop_ack',self.on_stop,10)
        self.complete_pub.publish(Bool(data=False))
        self.localized_pub.publish(Bool(data=False))
        self.build_cloud_messages()
        self.publish_map()
        self.create_timer(5.0,self.publish_map)
        self.status('WAITING FOR ALIGNMENT: use 3D Pose Estimate at robot location, drag its heading')

    def status(self,text,error=False):
        if error:self.get_logger().error(text)
        else:self.get_logger().info(text)
        msg=Marker()
        msg.header.frame_id='navigation_map'
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.ns='navigation_status';msg.id=0
        msg.type=Marker.TEXT_VIEW_FACING;msg.action=Marker.ADD
        msg.pose.position.x=float(self.data['center'][0])
        msg.pose.position.y=float(self.data['center'][1])
        msg.pose.position.z=float(np.nanmax(self.points[:,2])+1.0)
        msg.pose.orientation.w=1.0
        msg.scale.z=0.4
        msg.color.r=1.0 if error else 0.3
        msg.color.g=0.3 if error else 1.0
        msg.color.b=0.2;msg.color.a=1.0
        msg.text=text
        self.status_pub.publish(msg);self.state_pub.publish(String(data=text))

    def build_cloud_messages(self):
        header=Header(frame_id='navigation_map')
        self.map_msg=point_cloud2.create_cloud_xyz32(header,self.points)
        cost=self.data['data'][0];height=self.data['data'][3]
        valid=np.isfinite(height)&(cost>0)&(cost<=20)
        layer,row,col=np.where(valid)
        # Suppress duplicates from adjacent height slices.
        xyz=np.column_stack(((row-height.shape[1]//2)*self.data['resolution']+self.data['center'][0],
                             (col-height.shape[2]//2)*self.data['resolution']+self.data['center'][1],
                             height[layer,row,col]))
        xyz=np.unique(xyz,axis=0).astype(np.float32)
        self.free_msg=point_cloud2.create_cloud_xyz32(header,xyz)

    def publish_map(self):
        stamp=self.get_clock().now().to_msg()
        self.map_msg.header.stamp=stamp;self.free_msg.header.stamp=stamp
        self.map_pub.publish(self.map_msg);self.free_pub.publish(self.free_msg)

    def on_ready(self,msg):self.ready=bool(msg.data)

    def on_odom(self,msg):
        if msg.header.frame_id!='odom':return
        try:self.odom=pose_matrix(msg.pose.pose)
        except ValueError:return
        self.last_odom=time.monotonic()

    def on_cloud(self,msg):
        if msg.header.frame_id!='odom' or msg.width*msg.height<100:return
        raw=point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)
        if isinstance(raw,np.ndarray) and raw.dtype.names:
            points=np.column_stack([raw[name].ravel() for name in ('x','y','z')]).astype(np.float64)
        else:
            points=np.asarray(list(raw),dtype=np.float64)
        if points.ndim==2 and points.shape[1]==3 and len(points)>=100:
            self.cloud=points;self.last_cloud=time.monotonic()

    def fresh(self):
        return self.odom is not None and self.cloud is not None and time.monotonic()-self.last_odom<2.0 and time.monotonic()-self.last_cloud<2.0

    def on_initial_pose(self,msg):
        if self.active:
            self.status('ALIGNMENT REJECTED: a mission is active; wait for completion or restart',True);return
        if msg.header.frame_id!='navigation_map':
            self.status('POSE FRAME ERROR: RViz Fixed Frame must be navigation_map',True);return
        if not self.fresh():
            self.status('WAIT: live odometry/point cloud not ready; retry pose estimate',True);return
        try:
            estimate=pose_matrix(msg.pose.pose)@np.linalg.inv(self.odom)
            self.status('ALIGNING: matching live scan to the saved PCD...')
            matrix,fitness,rmse=register_cloud(self.cloud,self.points,estimate)
            self.map_from_odom=matrix
            inverse=np.linalg.inv(matrix)
            tf=TransformStamped()
            tf.header.frame_id='odom';tf.child_frame_id='navigation_map'
            tf.header.stamp=self.get_clock().now().to_msg()
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,inverse[:3,3])
            q=Rotation.from_matrix(inverse[:3,:3]).as_quat()
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,q)
            self.tf.sendTransform(tf)
            self.localized_pub.publish(Bool(data=True))
            (self.cache/'last_alignment.json').write_text(json.dumps({'map_from_odom':matrix.tolist(),'fitness':fitness,'rmse':rmse,
                'note':'Diagnostic only; not automatically reused on later runs'},indent=2))
            self.status('ALIGNED: fitness %.2f, RMSE %.2f m. Use 3D Nav Goal.'%(fitness,rmse))
        except Exception as error:
            self.status('ALIGNMENT FAILED: '+str(error),True)

    def on_goal(self,msg):
        if self.map_from_odom is None:
            self.status('GOAL NOT SENT: first set 3D Pose Estimate and wait for ALIGNED',True);return
        if not self.ready or not self.fresh():
            self.status('GOAL NOT SENT: robot/sensors are not ready',True);return
        try:
            target=np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z],dtype=float)
            if not np.isfinite(target).all():raise ValueError('Invalid goal coordinates')
            if msg.header.frame_id=='odom':
                target=transform_points(self.map_from_odom,target)
            elif msg.header.frame_id!='navigation_map':
                raise ValueError('Use navigation_map as RViz Fixed Frame')
            body=transform_points(self.map_from_odom,self.odom[:3,3])
            source_floor=body.copy();source_floor[2]-=0.5
            source,si=nearest_surface(self.data,source_floor,radius=0.8,height_tolerance=0.5)
            dest,gi=nearest_surface(self.data,target,radius=0.6,height_tolerance=0.4)
            self.status('PLANNING: searching the loaded PCD traversability map...')
            if np.array_equal(si,gi):
                route=np.array([body,dest+np.array([0,0,0.5])])
            else:
                if not self.planner.planner.plan(si,gi,False):
                    raise ValueError('No connected traversable path in this scanned map')
                raw=self.planner.planner.get_path_finder().get_result_matrix()
                route=self.planner._astar_route_to_map(raw)
            if len(route)<2 or not np.isfinite(route).all():raise ValueError('Invalid PCT route')
            route=transform_points(np.linalg.inv(self.map_from_odom),route)
            path=RosPath();path.header.frame_id='odom';path.header.stamp=self.get_clock().now().to_msg()
            for i,point in enumerate(route):
                pose=PoseStamped();pose.header=path.header
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,point)
                delta=route[min(i+1,len(route)-1)]-route[max(0,i-1)]
                yaw=math.atan2(delta[1],delta[0])
                pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
                path.poses.append(pose)
            goal=path.poses[-1]
            self.goal_stamp=(goal.header.stamp.sec,goal.header.stamp.nanosec)
            self.complete_pub.publish(Bool(data=False))
            self.goal_pub.publish(goal);self.path_pub.publish(path)
            self.active=True
            self.status('GOAL SENT: %d route points; target snapped %.2f m on the same floor'%(len(route),np.linalg.norm(dest-target)))
        except Exception as error:self.status('GOAL NOT SENT: '+str(error),True)

    def on_stop(self,msg):
        if self.active and (msg.header.stamp.sec,msg.header.stamp.nanosec)==self.goal_stamp:
            self.active=False
            self.complete_pub.publish(Bool(data=True))
            self.status('ARRIVED: choose another 3D Nav Goal')


def main():
    if len(sys.argv)<2:raise SystemExit('Usage: navigation_node.py CACHE')
    cache=sys.argv[1]
    rclpy.init(args=sys.argv[2:])
    node=MapNavigation(cache)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
