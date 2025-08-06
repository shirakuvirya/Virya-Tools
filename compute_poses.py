#!/usr/bin/env python3
import os
import csv
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

ODOM_TOPIC = "/dlio/odom_node/odom"

def extract_odometry_from_mcap(bag_path):
    odom_data = []
    try:
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topic_type = {t.name: t.type for t in reader.get_all_topics_and_types()}
        if ODOM_TOPIC not in topic_type:
            print(f"⚠️  {ODOM_TOPIC} not found in {bag_path}")
            return []

        msg_type = get_message(topic_type[ODOM_TOPIC])

        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic == ODOM_TOPIC:
                msg = deserialize_message(data, msg_type)
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                pos = msg.pose.pose.position
                odom_data.append([ts, pos.x, pos.y, pos.z])
    except Exception as e:
        print(f"❌ Error reading {bag_path}: {e}")

    return odom_data

def save_to_csv(data, folder_name, output_dir):
    if not data:
        print(f"⚠️  No data to save for {folder_name}")
        return

    filename = f"RTABMAP_{folder_name}.csv"
    save_path = os.path.join(output_dir, filename)
    with open(save_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "x", "y", "z"])
        writer.writerows(data)
    print(f"💾 Saved: {save_path}")

def process_all_mcap_files(root="."):
    for dirpath, _, filenames in os.walk(root):
        for fname in filenames:
            if fname.endswith(".mcap") and "rosbag2" in dirpath:
                full_path = os.path.join(dirpath, fname)
                parent_folder = os.path.basename(os.path.dirname(dirpath))
                print(f"\n🔍 Reading: {full_path}")
                data = extract_odometry_from_mcap(full_path)
                if data:
                    save_to_csv(data, parent_folder, root)

if __name__ == "__main__":
    process_all_mcap_files(".")
