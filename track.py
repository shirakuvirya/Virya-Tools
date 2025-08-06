#!/usr/bin/env python3

import os
import csv
import numpy as np

def read_odom_csv(csv_path):
    timestamps = []
    positions = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                ts = float(row['timestamp'])
                x = float(row['x'])
                y = float(row['y'])
                z = float(row['z'])
                timestamps.append(ts)
                positions.append((x, y, z))
            except Exception as e:
                print(f"⚠️  Skipping row in {csv_path} due to: {e}")
    
    return np.array(timestamps), np.array(positions)

def compute_metrics(timestamps, positions):
    if len(positions) < 2:
        return 0.0, 0.0, 0.0, []

    diffs = np.diff(positions, axis=0)
    dists = np.linalg.norm(diffs, axis=1)
    track_length = np.sum(dists)

    duration = timestamps[-1] - timestamps[0]
    avg_speed = track_length / duration if duration > 0 else 0.0

    jumps = [(i, d) for i, d in enumerate(dists) if d > 1.0]
    return track_length, duration, avg_speed, jumps

def process_folder(csv_path):
    timestamps, positions = read_odom_csv(csv_path)

    if len(timestamps) < 2:
        print(f"❌ Not enough data in: {csv_path}")
        return None

    track_length, duration, avg_speed, jumps = compute_metrics(timestamps, positions)

    print(f"📁 {os.path.dirname(csv_path)}")
    print(f"   - Distance    : {track_length:.2f} m")
    print(f"   - Duration    : {duration:.2f} s")
    print(f"   - Avg Speed   : {avg_speed:.2f} m/s")
    print(f"   - Jumps >1m   : {len(jumps)}")

    return {
        "folder": os.path.relpath(os.path.dirname(csv_path)),
        "length": track_length,
        "duration": duration,
        "speed": avg_speed,
        "jumps": len(jumps)
    }

def analyze_all_tracks(root="."):
    all_results = []

    for dirpath, _, filenames in os.walk(root):
        if "odom_from_mcap.csv" in filenames:
            csv_path = os.path.join(dirpath, "odom_from_mcap.csv")
            result = process_folder(csv_path)
            if result:
                all_results.append(result)

    if all_results:
        summary_path = os.path.join(root, "track_summary.txt")
        with open(summary_path, "w") as f:
            f.write(f"{'Folder':<50} {'Length (m)':<12} {'Duration (s)':<12} {'Speed (m/s)':<14} {'Pose Jumps'}\n")
            f.write("-" * 100 + "\n")
            for r in all_results:
                f.write(f"{r['folder']:<50} {r['length']:<12.2f} {r['duration']:<12.2f} {r['speed']:<14.2f} {r['jumps']}\n")
        print(f"\n✅ Summary saved to {summary_path}")
    else:
        print("❌ No valid odom_from_mcap.csv files found.")

if __name__ == "__main__":
    analyze_all_tracks(".")
