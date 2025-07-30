#!/usr/bin/env python3
"""
Copy a rosbag2 while discarding messages whose *header.stamp*
   is <= the previous one on the same topic.

Default topics cleaned:  /imu/data   /velodyne_points
Adjust TOPICS_TO_CLEAN as needed.
"""

import sys, pathlib
from collections import defaultdict
from rosbag2_py import SequentialReader, SequentialWriter
from rosbag2_py import StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ------------------------------------------------------------------
TOPICS_TO_CLEAN = {"/imu/data", "/velodyne_points"}      # edit if needed
STORAGE_ID = "sqlite3"
SERIAL_FMT = "cdr"
# ------------------------------------------------------------------

def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

def main(src, dst):
    if pathlib.Path(dst).exists():
        sys.exit(f"❌  Output dir {dst!r} already exists")

    # --- open input ---------------------------------------------------
    reader = SequentialReader()
    reader.open(StorageOptions(uri=src, storage_id=STORAGE_ID),
                ConverterOptions(SERIAL_FMT, SERIAL_FMT))

    topic_infos = {t.name: t for t in reader.get_all_topics_and_types()}
    deserializers = {
        name: get_message(info.type) for name, info in topic_infos.items()
        if name in TOPICS_TO_CLEAN
    }

    # --- open output --------------------------------------------------
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=dst, storage_id=STORAGE_ID),
                ConverterOptions(SERIAL_FMT, SERIAL_FMT))
    for info in topic_infos.values():
        writer.create_topic(info)

    last_stamp = defaultdict(lambda: -1)
    dropped = defaultdict(int)
    total = 0

    print(f"Filtering topics: {', '.join(TOPICS_TO_CLEAN)}")
    while reader.has_next():
        topic, data, t_bag = reader.read_next()
        total += 1
        if topic in TOPICS_TO_CLEAN:
            msg = deserialize_message(data, deserializers[topic])
            cur = stamp_ns(msg)
            if cur <= last_stamp[topic]:
                dropped[topic] += 1
                continue          # skip this message
            last_stamp[topic] = cur
        writer.write(topic, data, t_bag)

    print("\n✓ done:")
    kept = total - sum(dropped.values())
    print(f"  kept     : {kept:,}")
    for tp, n in dropped.items():
        print(f"  dropped  : {n:,}  ({tp})")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit("usage: rosbag_drop_bad_frames.py <in_bag_dir> <out_bag_dir>")
    main(sys.argv[1], sys.argv[2])
