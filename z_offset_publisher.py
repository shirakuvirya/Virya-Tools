#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointField
import math
import time

class ZOffsetRepublisher(Node):
    def __init__(self):
        super().__init__('z_offset_pointcloud_republisher')
        self.sub = self.create_subscription(PointCloud2, '/velodyne_points', self.callback, 10)
        self.pub = self.create_publisher(PointCloud2, '/modified_velodyne_points', 10)
        self.start_time = time.time()

    def callback(self, msg):
        now = time.time() - self.start_time
        z_offset = 0.25 * math.sin(0.5 * now)  # 25 cm oscillation

        # Modify points
        points = list(pc2.read_points(msg, field_names=["x", "y", "z", "intensity"], skip_nans=True))
        modified_points = [(x, y, z + z_offset, i) for x, y, z, i in points]

        modified_cloud = pc2.create_cloud(msg.header, msg.fields, modified_points)
        modified_cloud.header.stamp = msg.header.stamp
        modified_cloud.header.frame_id = msg.header.frame_id
        self.pub.publish(modified_cloud)

def main(args=None):
    rclpy.init(args=args)
    node = ZOffsetRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

