#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import signal
import sys
from datetime import datetime

class LocalizationLogger(Node):
    def __init__(self):
        super().__init__('localization_logger')
        self.sub = self.create_subscription(PoseStamped, '/localization_pose', self.callback, 10)
        self.data = []

        self.get_logger().info("Subscribed to /localization_pose. Press CTRL+C to stop and save CSV.")

    def callback(self, msg):
        position = msg.pose.position
        timestamp = self.get_clock().now().to_msg()
        self.data.append([
            timestamp.sec + timestamp.nanosec * 1e-9,
            position.x,
            position.y,
            position.z
        ])

    def save_to_csv(self, filename='localization_data.csv'):
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'z'])
            writer.writerows(self.data)
        self.get_logger().info(f"Saved data to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationLogger()

    def signal_handler(sig, frame):
        node.get_logger().info("CTRL+C detected. Saving CSV and shutting down.")
        node.save_to_csv()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
