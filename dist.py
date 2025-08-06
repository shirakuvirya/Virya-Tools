#!/usr/bin/env python3

import os
import subprocess
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from threading import Event

odom_messages = []

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/dlio/odom_node/odom',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        odom_messages.append((
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ))

def find_all_mcap_files(root_dir):
    mcap_paths = []
    for dirpath, _, filenames in os.walk(root_dir):
        for f in filenames:
            if f.endswith('.mcap') and "rosbag2" in dirpath:
                full_path = os.path.join(dirpath, f)
                mcap_paths.append(full_path)
    return mcap_paths

def play_mcap_file(mcap_file):
    print(f"\n▶️  Playing: {mcap_file}")
    try:
        return subprocess.Popen(['ros2', 'bag', 'play', mcap_file])
    except Exception as e:
        print(f"❌ Failed to play {mcap_file}: {e}")
        return None

def record_odom_from_bag(mcap_file, duration=15):
    rclpy.init()
    node = OdomSubscriber()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    bag_process = play_mcap_file(mcap_file)
    print("⏳ Waiting for messages...")
    start_time = time.time()

    try:
        while time.time() - start_time < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if bag_process:
            bag_process.terminate()
            bag_process.wait()
        node.destroy_node()
        rclpy.shutdown()

    print(f"✅ Recorded {len(odom_messages)} messages from {mcap_file}")
    return odom_messages.copy()

def save_odom_to_csv(messages, save_path):
    if not messages:
        print("⚠️  No odometry data to save.")
        return
    import csv
    with open(save_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'x', 'y', 'z'])
        writer.writerows(messages)
    print(f"💾 Saved odometry data to: {save_path}")

def main():
    root_dir = "."
    mcap_files = find_all_mcap_files(root_dir)

    if not mcap_files:
        print("❌ No .mcap files found.")
        return

    print(f"🔍 Found {len(mcap_files)} .mcap files.\n")

    for mcap in mcap_files:
        odom_messages.clear()
        try:
            messages = record_odom_from_bag(mcap)
            save_path = os.path.join(os.path.dirname(mcap), "odom_track.csv")
            save_odom_to_csv(messages, save_path)
        except Exception as e:
            print(f"❌ Error during processing {mcap}: {e}")

    print("\n🏁 All files processed.")

if __name__ == "__main__":
    main()
