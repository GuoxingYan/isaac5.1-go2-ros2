#!/usr/bin/env python3
"""Visualize live Livox IMU vectors as RViz arrows."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray


class ImuRviz(Node):
    def __init__(self):
        super().__init__("livox_imu_rviz")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(MarkerArray, "/livox/imu_vectors", qos)
        self.subscription = self.create_subscription(Imu, "/livox/imu", self.on_imu, qos)

    def on_imu(self, msg):
        vectors = (
            ("linear_acceleration", msg.linear_acceleration, (0.1, 0.0, 1.0), 0.5),
            ("angular_velocity", msg.angular_velocity, (1.0, 0.35, 0.05), 2.0),
        )
        markers = MarkerArray()
        for marker_id, (name, vector, color, scale) in enumerate(vectors):
            marker = Marker()
            marker.header = msg.header
            marker.ns = name
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.025
            marker.scale.y = 0.055
            marker.scale.z = 0.08
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            marker.lifetime.sec = 0
            marker.points = [
                self._point(0.0, 0.0, 0.0),
                self._point(vector.x * scale, vector.y * scale, vector.z * scale),
            ]
            markers.markers.append(marker)
        self.publisher.publish(markers)

    @staticmethod
    def _point(x, y, z):
        from geometry_msgs.msg import Point
        return Point(x=float(x), y=float(y), z=float(z))


def main():
    rclpy.init()
    node = ImuRviz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
