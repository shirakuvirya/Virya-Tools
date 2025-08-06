#!/usr/bin/env python3
"""
compute.py

Recursively finds, in each subfolder:
  • one *_top_processes.csv   (process snapshot: name,cpu,ram…)
  • one *_system.csv         (per‐core CPU usage)

And writes in each folder a structured_compute_summary.txt:

  - Total CPU       : avg across cores
  - SLAM CPU        : % of total capacity eaten by SLAM
  - SLAM + Viz CPU  : % of total capacity eaten by SLAM+Viz
"""

import os, pandas as pd

# tweak these to match your CSV’s “name” column exactly:
SLAM_NAMES = [
    "dlio_map_node", "dlio_odom_node",
    "icp_odometry", "imu_to_tf",
    "pointcloud_to_depthimage",
    "robot_state_publisher", "ros2",
    "rtabmap"
]
VIZ_NAMES = ["rtabmap_viz", "rviz2", "rviz"]

def find_csv_pairs(root="."):
    for dirpath, _, files in os.walk(root):
        top = next((os.path.join(dirpath, f)
                    for f in files if f.endswith("_top_processes.csv")), None)
        sys = next((os.path.join(dirpath, f)
                    for f in files if f.endswith("_system.csv")), None)
        if top and sys:
            yield dirpath, top, sys

def summarize_folder(folder, top_csv, sys_csv):
    top_df = pd.read_csv(top_csv)
    sys_df = pd.read_csv(sys_csv)

    # infer real # of cores from your system snapshot
    ncores = len(sys_df)

    # 1) total CPU % (average per‐core usage)
    total_pct = sys_df["cpu"].mean()

    # 2) SLAM only: sum of their %cpu ÷ #cores → % of total capacity
    slam_sum = top_df.loc[top_df["name"].isin(SLAM_NAMES), "cpu"].sum()
    slam_pct = slam_sum / ncores

    # 3) SLAM + viz
    viz_sum      = top_df.loc[top_df["name"].isin(VIZ_NAMES), "cpu"].sum()
    slam_viz_pct = (slam_sum + viz_sum) / ncores

    return total_pct, slam_pct, slam_viz_pct

def write_summary(folder, totals):
    total, slam, slam_viz = totals
    txt = (
        f"Summary for {os.path.basename(folder)}:\n"
        f" - Total CPU       : {total:.2f}% of capacity\n"
        f" - SLAM CPU        : {slam:.2f}% of capacity\n"
        f" - SLAM + Viz CPU  : {slam_viz:.2f}% of capacity\n"
    )
    with open(os.path.join(folder, "structured_compute_summary.txt"), "w") as f:
        f.write(txt)

def main():
    for folder, top_csv, sys_csv in find_csv_pairs():
        try:
            totals = summarize_folder(folder, top_csv, sys_csv)
            write_summary(folder, totals)
            print(f"✔️  Wrote summary in {folder}")
        except Exception as e:
            print(f"❌  Failed in {folder}: {e}")

if __name__ == "__main__":
    main()
