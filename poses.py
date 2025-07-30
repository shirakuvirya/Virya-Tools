#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
from datetime import datetime

class LocalizationLogger(Node):
    def __init__(self):
        super().__init__('localization_logger')
        self.sub = self.create_subscription(PoseStamped, '/localization_pose', self.callback, 10)
        self.data = []
        self.filename = f'localization_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        self.get_logger().info("Subscribed to /localization_pose. Press CTRL+C to stop and save CSV.")

        # Register shutdown hook
        rclpy.get_default_context().on_shutdown(self.on_shutdown)

    def callback(self, msg):
        pos = msg.pose.position
        timestamp = self.get_clock().now().to_msg()
        self.data.append([
            timestamp.sec + timestamp.nanosec * 1e-9,
            pos.x,
            pos.y,
            pos.z
        ])

    def on_shutdown(self):
        self.get_logger().info("Shutting down. Writing to CSV...")
        self.save_to_csv()
        self.get_logger().info(f"Saved data to {self.filename}")

    def save_to_csv(self):
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'z'])
            writer.writerows(self.data)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Also ensure clean shutdown here
        node.on_shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
