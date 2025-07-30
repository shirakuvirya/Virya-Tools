#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt
import time


class TFZLogger(Node):
    def __init__(self):
        super().__init__('tf_z_logger')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.data = {"time": [], "z": [], "x": [], "y": []}
        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.duration = 60  # seconds

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', now)
            t = time.time() - self.start_time
            z = trans.transform.translation.z
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            self.get_logger().info(f"[{t:.2f}s] z = {z:.4f}")
            self.data["time"].append(t)
            self.data["z"].append(z)
            self.data["x"].append(x)
            self.data["y"].append(y)

            if t >= self.duration:
                self.timer.cancel()
                self.get_logger().info("Finished recording. Plotting...")
                self.plot()

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")

    def plot(self):
        times = self.data["time"]
        plt.figure(figsize=(10, 6))
        plt.plot(times, self.data["z"], label='Z (vertical)', color='blue')
        plt.plot(times, self.data["x"], label='X', linestyle='--')
        plt.plot(times, self.data["y"], label='Y', linestyle='--')
        plt.xlabel("Time (s)")
        plt.ylabel("Position (meters)")
        plt.grid()
        plt.legend()
        plt.title("Robot Z Position Over Time (TF: map → base_link)")
        plt.tight_layout()
        plt.show()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TFZLogger()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

