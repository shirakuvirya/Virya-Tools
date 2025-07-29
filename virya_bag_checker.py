#!/usr/bin/env python3
# Author: Shivaram Kumar Jagannathan

import os
import sys
import csv
from collections import defaultdict
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

FREQ_TOLERANCES = {
    "/velodyne_points": (10.0, 0.5),
    "/imu/data": (100.0, 2.0),
    "/camera/camera/color/image_raw": (30.0, 1.0),
    "/camera/camera/depth/camera_info": (30.0, 1.0),
    "/camera/camera/depth/image_rect_raw": (30.0, 1.0),
    "/camera/camera/color/camera_info": (30.0, 1.0),
}

COMPLETENESS_THRESHOLD = 0.95
COMPRESSION_RATIO_MIN = 0.2

def check_bag(bag_path):
    print(f"\n🔍 Starting check for bag: {bag_path}")

    # Open reader
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_timestamps = defaultdict(list)

    print("📥 Reading messages and timestamps...\n")
    msg_count = 0
    topic_msg_counts = defaultdict(int)

    try:
        import tqdm
        use_progress = True
    except ImportError:
        use_progress = False

    # Collect timestamps
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg_count += 1
        if topic in FREQ_TOLERANCES:
            msg_timestamps[topic].append(t / 1e9)
            topic_msg_counts[topic] += 1

    print(f"📦 Total messages read: {msg_count:,}")
    print(f"📌 Topics of interest found: {len(msg_timestamps)}\n")

    results = []
    bag_pass = True

    for topic, (expected_freq, tolerance) in FREQ_TOLERANCES.items():
        print(f"🔎 Checking topic: {topic}")
        stamps = msg_timestamps[topic]

        if len(stamps) < 2:
            print(f"  ❌ Not enough messages for {topic} ({len(stamps)} found)")
            results.append({
                "topic": topic,
                "frequency": "0.0",
                "expected": f"{expected_freq}±{tolerance}",
                "freq_pass": "FAIL",
                "completeness": "0.0%",
                "complete_pass": "FAIL",
                "max_gap": "-",
                "timestamp_order_pass": "FAIL"
            })
            bag_pass = False
            continue

        duration = max(stamps) - min(stamps)
        freq = len(stamps) / duration
        completeness = len(stamps) / (expected_freq * duration)
        time_diffs = [stamps[i+1] - stamps[i] for i in range(len(stamps)-1)]
        max_gap = max(time_diffs) if time_diffs else 0.0

        freq_ok = (expected_freq - tolerance) <= freq <= (expected_freq + tolerance)
        comp_ok = completeness >= COMPLETENESS_THRESHOLD
        in_order = all(stamps[i] <= stamps[i+1] for i in range(len(stamps) - 1))

        # Print verdict with ✔ / ❌
        print(f"  📊 Duration: {duration:.2f}s, Msgs: {len(stamps)}, Freq: {freq:.2f} Hz", end=' ')
        print("✔" if freq_ok else "❌")
        print(f"  🧮 Completeness: {completeness*100:.1f}%", end=' ')
        print("✔" if comp_ok else "❌")
        print(f"  ⏱️ Max time gap between msgs: {max_gap:.3f}s")
        print(f"  📏 Timestamps ordered: {'✔' if in_order else '❌'}")

        results.append({
            "topic": topic,
            "frequency": f"{freq:.2f}",
            "expected": f"{expected_freq}±{tolerance}",
            "freq_pass": "PASS" if freq_ok else "FAIL",
            "completeness": f"{completeness*100:.2f}%",
            "complete_pass": "PASS" if comp_ok else "FAIL",
            "max_gap": f"{max_gap:.3f}s",
            "timestamp_order_pass": "PASS" if in_order else "FAIL"
        })

        if not (freq_ok and comp_ok and in_order):
            bag_pass = False

    # Save output to CSV
    out_dir = bag_path if os.path.isdir(bag_path) else os.path.dirname(os.path.abspath(bag_path))
    csv_file = os.path.join(out_dir, "data_sanity_check_parameters.csv")
    with open(csv_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)

    print("\n📝 CSV results written to:", csv_file)

    # Final status
    print("\n📈 FINAL BAG STATUS")
    if bag_pass:
        print("✅ Bag PASSES all sanity checks.")
    else:
        print("❌ Bag FAILED sanity checks. Please re-record or verify data integrity.")

    return bag_pass


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python virya_bag_checker.py <path_to_bag_or_directory>")
        sys.exit(1)

    bag_path = sys.argv[1]
    if not os.path.exists(bag_path):
        print(f"❌ Error: Path '{bag_path}' does not exist.")
        sys.exit(1)

    check_bag(bag_path)
