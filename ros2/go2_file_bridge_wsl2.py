#!/usr/bin/env python3
"""
Ubuntu ROS2 file bridge: reads sim data from shared files, publishes as ROS2 topics
and subscribes to cmd_vel to control the robot.
Run: /usr/bin/python3 go2_file_bridge_wsl2.py
"""
import json, time, os, sys
from pathlib import Path
PROFILE=json.loads((Path(__file__).resolve().parents[1]/"cfg/our_go2_sensors.json").read_text())
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo, Imu
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from PIL import Image as PILImage
import numpy as np

DATA_DIR = os.environ.get("GO2_DATA_DIR", os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "outputs", "our-go2-bridge"))
CMD_VEL_FILE = os.path.join(DATA_DIR, "cmd_vel.json")


class Go2Bridge(Node):
    def __init__(self):
        super().__init__("go2_file_bridge")
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/unitree_go2/odom", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/unitree_go2/pose", 10)
        self.img_pub = self.create_publisher(Image, "/unitree_go2/front_cam/color_image", 1)
        self.depth_pub = self.create_publisher(Image, "/unitree_go2/front_cam/depth_image", 1)
        self.semseg_pub = self.create_publisher(Image, "/unitree_go2/front_cam/semantic_segmentation_image", 1)
        self.lidar_pub = self.create_publisher(PointCloud2, "/unitree_go2/lidar/point_cloud", 1)
        self.cam_info_pub = self.create_publisher(CameraInfo, "/unitree_go2/front_cam/info", 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/unitree_go2/front_cam/depth_info", 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.imu_pub = self.create_publisher(Imu, "/livox/imu", 200)
        self.livox_pub = self.create_publisher(PointCloud2, "/livox/lidar", 1)
        self.last_imu_stamp=0.
        self.last_camera_time=None
        self.last_lidar_time=None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        transforms=[]
        for child, xyz, quat in [
            ("unitree_go2/lidar", PROFILE["lidar"]["position"], [0.,0.,0.,1.]),
            ("unitree_go2/front_cam", PROFILE["camera"]["position"], [-0.5,0.5,-0.5,0.5]),
            ("unitree_go2/d435_depth_optical", PROFILE["camera"]["position"], [-0.5,0.5,-0.5,0.5]),
            ("unitree_go2/mid360_imu", (np.array(PROFILE["lidar"]["position"])+np.array(PROFILE["imu"]["position_in_lidar"])).tolist(), [0.,0.,0.,1.])]:
            t=TransformStamped()
            t.header.stamp=self.get_clock().now().to_msg()
            t.header.frame_id="unitree_go2/base_link"
            t.child_frame_id=child
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=xyz
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=quat
            transforms.append(t)
        self.static_tf.sendTransform(transforms)
        self.bridge = self
        self.last_frame = -1
        self.latest_cmd_vel = None  # store for debug

        # Subscriber: cmd_vel from ROS2 (e.g. from rviz2 2D Nav Goal or teleop)
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/unitree_go2/cmd_vel", self._cmd_vel_callback, 10
        )

        self.get_logger().info(
            "Go2 Bridge started\n"
            "  Publishing:  /unitree_go2/odom, /pose, /front_cam/*, /lidar/point_cloud\n"
            "  Subscribing: /unitree_go2/cmd_vel"
        )

    @staticmethod
    def cv2_to_imgmsg(arr, encoding):
        arr = np.ascontiguousarray(arr)
        msg = Image()
        msg.height, msg.width = arr.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = arr.strides[0]
        msg.data = arr.tobytes()
        return msg

    def _cmd_vel_callback(self, msg: Twist):
        """Write received cmd_vel to shared file for simulation side to read."""
        cmd = {
            "linear_x": msg.linear.x,
            "linear_y": msg.linear.y,
            "angular_z": msg.angular.z,
            "timestamp": time.time(),
        }
        try:
            os.makedirs(DATA_DIR, exist_ok=True)
            with open(CMD_VEL_FILE + ".tmp", "w") as f:
                json.dump(cmd, f)
            os.replace(CMD_VEL_FILE + ".tmp", CMD_VEL_FILE)
            self.latest_cmd_vel = cmd
            self.get_logger().debug(
                f"cmd_vel received: lin=({msg.linear.x:.2f}, {msg.linear.y:.2f}), ang={msg.angular.z:.2f}"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to write cmd_vel: {e}")

    def publish_data(self):
        json_path = os.path.join(DATA_DIR, "data.json")
        if not os.path.exists(json_path):
            return

        try:
            with open(json_path, "r") as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError):
            return

        frame = data.get("frame", 0)
        if frame < self.last_frame:
            self.last_imu_stamp=0.
            self.last_camera_time=None
            self.last_lidar_time=None
        if frame == self.last_frame:
            return
        self.last_frame = frame

        sim_stamp=float(data.get('simulation_time',0.))
        now = Time(sec=int(sim_stamp),nanosec=int((sim_stamp%1)*1e9))
        self.clock_pub.publish(Clock(clock=now))
        odom_data = data.get("odom", {})
        pos = odom_data.get("position", [0, 0, 0])
        ori = odom_data.get("orientation", [0, 0, 0, 1])

        # --- Odometry ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "unitree_go2/base_link"
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        odom.pose.pose.orientation.x = ori[0]
        odom.pose.pose.orientation.y = ori[1]
        odom.pose.pose.orientation.z = ori[2]
        odom.pose.pose.orientation.w = ori[3]
        self.odom_pub.publish(odom)

        # --- TF ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "unitree_go2/base_link"
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]
        t.transform.rotation.x = ori[0]
        t.transform.rotation.y = ori[1]
        t.transform.rotation.z = ori[2]
        t.transform.rotation.w = ori[3]
        self.tf_broadcaster.sendTransform(t)

        # --- Pose ---
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        # --- Camera Image ---
        camera_fresh=data.get("camera_time") != self.last_camera_time
        if camera_fresh:
            self.last_camera_time=data.get("camera_time")
        img_name = data.get("color_image") if camera_fresh else None
        if img_name:
            img_path = os.path.join(DATA_DIR, img_name)
            if os.path.exists(img_path):
                try:
                    cv_img = np.asarray(PILImage.open(img_path).convert("RGB"))
                    if cv_img is not None:
                        ros_img = self.bridge.cv2_to_imgmsg(cv_img, "rgb8")
                        camera_stamp=float(data.get('camera_time',sim_stamp))
                        ros_img.header.stamp = Time(sec=int(camera_stamp),nanosec=int((camera_stamp%1)*1e9))
                        ros_img.header.frame_id = "unitree_go2/front_cam"
                        self.img_pub.publish(ros_img)

                        # Camera info
                        h, w = cv_img.shape[:2]
                        if h > 0 and w > 0:
                            cam_info = CameraInfo()
                            cam_info.header = ros_img.header
                            cam_info.height = h
                            cam_info.width = w
                            cam_info.distortion_model = "plumb_bob"
                            k = np.asarray(data.get("intrinsics", [[w,0,w/2],[0,w,h/2],[0,0,1]]), dtype=float)
                            cam_info.k = k.ravel().tolist()
                            cam_info.r = np.eye(3).ravel().tolist()
                            cam_info.p = np.column_stack([k,np.zeros(3)]).ravel().tolist()
                            self.cam_info_pub.publish(cam_info)
                except Exception as e:
                    self.get_logger().warn(f"Camera error: {e}")

        # --- Depth Image ---
        depth_name = data.get("depth_image") if camera_fresh else None
        if depth_name:
            depth_path = os.path.join(DATA_DIR, depth_name)
            if os.path.exists(depth_path):
                try:
                    depth_cv = np.load(depth_path)
                    if depth_cv is not None:
                        ros_depth = self.bridge.cv2_to_imgmsg(depth_cv, "32FC1")
                        camera_stamp=float(data.get('camera_time',sim_stamp))
                        ros_depth.header.stamp = Time(sec=int(camera_stamp),nanosec=int((camera_stamp%1)*1e9))
                        ros_depth.header.frame_id = "unitree_go2/d435_depth_optical"
                        info = CameraInfo()
                        info.header=ros_depth.header
                        info.height,info.width=depth_cv.shape
                        k=np.asarray(data['depth_intrinsics'],dtype=float)
                        info.k=k.ravel().tolist()
                        info.r=np.eye(3).ravel().tolist()
                        info.p=np.column_stack([k,np.zeros(3)]).ravel().tolist()
                        self.depth_info_pub.publish(info)
                        self.depth_pub.publish(ros_depth)
                except Exception as e:
                    self.get_logger().warn(f"Depth error: {e}")

        # --- Semantic Segmentation ---
        sem_name = data.get("semantic") if camera_fresh else None
        if sem_name:
            sem_path = os.path.join(DATA_DIR, sem_name)
            if os.path.exists(sem_path):
                try:
                    sem_cv = np.load(sem_path).astype(np.int32)
                    if sem_cv is not None:
                        ros_sem = self.bridge.cv2_to_imgmsg(sem_cv, "32SC1")
                        ros_sem.header.stamp = ros_depth.header.stamp
                        ros_sem.header.frame_id = "unitree_go2/d435_depth_optical"
                        self.semseg_pub.publish(ros_sem)
                except Exception as e:
                    self.get_logger().warn(f"Semantic error: {e}")

        # --- LiDAR Point Cloud ---
        lidar_fresh=data.get("lidar_time") != self.last_lidar_time
        if lidar_fresh:
            self.last_lidar_time=data.get("lidar_time")
        pc_name = data.get("point_cloud") if lidar_fresh else None
        if pc_name:
            pc_path = os.path.join(DATA_DIR, pc_name)
            if os.path.exists(pc_path):
                try:
                    pts = np.load(pc_path)
                    if len(pts) > 0:
                        fields = [
                            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                        ]
                        from std_msgs.msg import Header
                        lidar_stamp=float(data.get('lidar_time',sim_stamp))
                        header = Header(stamp=Time(sec=int(lidar_stamp),nanosec=int((lidar_stamp%1)*1e9)), frame_id="unitree_go2/lidar")
                        xyz = np.ascontiguousarray(pts[:, :3], dtype=np.float32)
                        pc2 = PointCloud2()
                        pc2.header = header
                        pc2.height = 1
                        pc2.width = len(xyz)
                        pc2.fields = fields
                        pc2.is_bigendian = False
                        pc2.point_step = 12
                        pc2.row_step = pc2.point_step * pc2.width
                        pc2.data = xyz.tobytes()
                        pc2.is_dense = bool(np.isfinite(xyz).all())
                        self.lidar_pub.publish(pc2)
                        self.livox_pub.publish(pc2)
                except Exception as e:
                    self.get_logger().warn(f"Lidar error: {e}")

        # 每个 5ms 物理采样只发布一次；不虚构姿态或硬件标定协方差。
        for sample in data.get('imu_samples',[]):
            if sample['robot'] != 0 or sample['time'] <= self.last_imu_stamp:
                continue
            self.last_imu_stamp=sample['time']
            msg=Imu()
            msg.header.frame_id='unitree_go2/mid360_imu'
            msg.header.stamp.sec=int(sample['time'])
            msg.header.stamp.nanosec=int((sample['time']%1)*1e9)
            msg.orientation_covariance[0]=-1.
            msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=sample['angular_velocity']
            msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=sample['linear_acceleration']
            self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = Go2Bridge()
    import threading
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        while rclpy.ok():
            node.publish_data()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
