#!/usr/bin/env python3
"""Collect low-overhead FAST-LIVO2 input/output topic evidence to JSON."""
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry


class Probe(Node):
    def __init__(self, output):
        super().__init__("mapping_acceptance_probe")
        self.output = Path(output)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.started = time.monotonic()
        self.counts = {}
        self.details = {}
        self.samples = {}
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=256,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, "/livox/lidar", self.on_lidar, qos)
        self.create_subscription(Imu, "/livox/imu", self.on_imu, qos)
        self.create_subscription(Odometry, "/aft_mapped_to_init", self.on_odom, 20)
        self.create_subscription(PointCloud2, "/cloud_registered_local", self.on_cloud, 10)

    def tick(self, topic, msg):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.setdefault(topic, []).append(float(stamp))
        if len(self.samples[topic]) > 10000:
            del self.samples[topic][:-5000]

    def on_lidar(self, msg):
        topic = "/livox/lidar"
        self.tick(topic, msg)
        self.details[topic] = {
            "type": "sensor_msgs/msg/PointCloud2", "frame_id": msg.header.frame_id,
            "width": msg.width, "height": msg.height, "point_step": msg.point_step,
            "row_step": msg.row_step,
            "fields": [{"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count} for f in msg.fields],
        }

    def on_imu(self, msg):
        topic = "/livox/imu"
        self.tick(topic, msg)
        self.details[topic] = {"type": "sensor_msgs/msg/Imu", "frame_id": msg.header.frame_id}

    def on_odom(self, msg):
        topic = "/aft_mapped_to_init"
        self.tick(topic, msg)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.details[topic] = {
            "type": "nav_msgs/msg/Odometry", "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "position": [float(p.x), float(p.y), float(p.z)],
            "orientation_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
        }

    def on_cloud(self, msg):
        topic = "/cloud_registered_local"
        self.tick(topic, msg)
        self.details[topic] = {
            "type": "sensor_msgs/msg/PointCloud2", "frame_id": msg.header.frame_id,
            "width": msg.width, "height": msg.height, "point_step": msg.point_step,
            "fields": [{"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count} for f in msg.fields],
        }

    def snapshot(self, status="running"):
        elapsed = max(time.monotonic() - self.started, 1e-6)
        result = {"status": status, "elapsed_wall_s": elapsed, "counts": self.counts,
                  "details": self.details, "frequency_hz": {}, "stamp_ranges_s": {}}
        for topic, times in self.samples.items():
            if times:
                result["stamp_ranges_s"][topic] = [min(times), max(times)]
            if len(times) >= 2 and max(times) > min(times):
                result["frequency_hz"][topic] = (len(times) - 1) / (max(times) - min(times))
        self.output.write_text(json.dumps(result, indent=2), encoding="utf-8")


def main():
    output = Path(os.environ["GO2_PROBE_OUTPUT"])
    duration = float(os.environ.get("GO2_PROBE_SECONDS", "3600"))
    rclpy.init()
    node = Probe(output)
    stop = time.monotonic() + duration
    try:
        while rclpy.ok() and time.monotonic() < stop:
            rclpy.spin_once(node, timeout_sec=0.1)
            if int(time.monotonic() - node.started) % 2 == 0:
                node.snapshot()
        node.snapshot("stopped")
    except (KeyboardInterrupt, ExternalShutdownException):
        node.snapshot("interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
