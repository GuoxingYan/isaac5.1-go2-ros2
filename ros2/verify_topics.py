"""Record actual received ROS messages, validate depth encoding and command roundtrip."""
import json
import os
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist

rclpy.init()
node=Node('go2_acceptance_probe')
counts={}
details={}
imu_times=[]
def receive(name,msg):
    counts[name]=counts.get(name,0)+1
    if isinstance(msg,Image):
        details[name]={'encoding':msg.encoding,'width':msg.width,'height':msg.height,'bytes':len(msg.data)}
    elif isinstance(msg,PointCloud2):
        details[name]={'points':msg.width*msg.height,'frame':msg.header.frame_id}
    elif isinstance(msg,Imu):
        stamp=float(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        imu_times.append(stamp)
        details[name]={'frame':msg.header.frame_id,'time':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'acceleration':[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z], 'gyro':[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z], 'orientation_unavailable':bool(msg.orientation_covariance[0]==-1.)}
    elif isinstance(msg,Odometry):
        details[name]={'position':[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]}
subscriptions=[]
for name,typ,topic in [('rgb',Image,'/unitree_go2/front_cam/color_image'),('depth',Image,'/unitree_go2/front_cam/depth_image'),('semantic',Image,'/unitree_go2/front_cam/semantic_segmentation_image'),('lidar',PointCloud2,'/unitree_go2/lidar/point_cloud'),('odom',Odometry,'/unitree_go2/odom'),('info',CameraInfo,'/unitree_go2/front_cam/info'),('tf',TFMessage,'/tf'),('imu',Imu,'/livox/imu'),('depth_info',CameraInfo,'/unitree_go2/front_cam/depth_info'),('livox',PointCloud2,'/livox/lidar')]:
    counts[name]=0
    subscriptions.append(node.create_subscription(typ,topic,lambda m,n=name:receive(n,m),1000 if typ is Imu else 10))
pub=node.create_publisher(Twist,'/unitree_go2/cmd_vel',10)
started=time.monotonic()
output=Path(os.environ.get('GO2_REPORT_DIR','reports/our-go2'))
output.mkdir(parents=True,exist_ok=True)
try:
    while time.monotonic()-started<float(os.environ.get('GO2_PROBE_SECONDS','600')):
        rclpy.spin_once(node,timeout_sec=.1)
        request=Path('outputs/ros_command_request.json')
        if request.exists():
            try:
                req=json.loads(request.read_text())
            except (OSError, json.JSONDecodeError):
                req={'timestamp':0}
            if time.time()-req['timestamp']<1.8:
                command=Twist()
                command.linear.x=.5
                pub.publish(command)
        if int(time.monotonic()-started)%3==0:
            # Zero command verifies transport without interfering with movement tests.
            if os.environ.get('GO2_SEND_STOP')=='1': pub.publish(Twist())
        (output/'ros_topics.json').write_text(json.dumps({'counts':counts,'details':details,'elapsed_s':time.monotonic()-started,'imu_times':imu_times},indent=2))
finally:
    node.destroy_node();rclpy.shutdown()
