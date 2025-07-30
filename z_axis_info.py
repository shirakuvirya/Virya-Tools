import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt
import time
import threading

class ZAxisLogger(Node):
    def __init__(self):
        super().__init__('z_axis_logger')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timestamps = []
        self.z_values = []

        self.running = True
        self.thread = threading.Thread(target=self.log_z)
        self.thread.start()

    def log_z(self):
        self.get_logger().info("Logging Z-axis data from map → base_link...")
        while rclpy.ok() and self.running:
            try:
                now = rclpy.time.Time()
                trans: TransformStamped = self.tf_buffer.lookup_transform(
                    'map', 'base_link', now, rclpy.duration.Duration(seconds=0.5))

                z = trans.transform.translation.z
                t = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9

                self.timestamps.append(t)
                self.z_values.append(z)

                time.sleep(0.05)  # 20 Hz

            except Exception as e:
                self.get_logger().warn(f"Transform not available: {e}")
                time.sleep(0.1)

    def stop_and_plot(self):
        self.running = False
        self.thread.join()

        self.get_logger().info("Plotting Z over time...")

        plt.figure()
        plt.plot(self.timestamps, self.z_values, marker='.')
        plt.xlabel("Time [s]")
        plt.ylabel("Z Position [m]")
        plt.title("Z-Axis Position (map → base_link)")
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = ZAxisLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_and_plot()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

