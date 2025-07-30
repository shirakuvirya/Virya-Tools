#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt
import time

class TransformRecorder(Node):
    def __init__(self):
        super().__init__('transform_recorder')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.z_values = []
        self.timestamps = []
        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.record_transform)

    def record_transform(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                now,
                rclpy.duration.Duration(seconds=1.0)
            )
            z = trans.transform.translation.z
            self.z_values.append(z)
            self.timestamps.append(time.time() - self.start_time)
            self.get_logger().info(f"Z: {z:.3f} m")
        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")

    def save_and_plot(self):
        self.get_logger().info("Saving and plotting Z-axis data...")
        plt.plot(self.timestamps, self.z_values)
        plt.xlabel('Time (s)')
        plt.ylabel('Z Position (m)')
        plt.title('Z-axis Drift over Time (map → base_link)')
        plt.grid(True)
        plt.savefig("z_drift_plot.png")
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = TransformRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_and_plot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

