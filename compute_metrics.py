#!/usr/bin/env python3

import os
import re
import pandas as pd
from collections import defaultdict

def find_matching_logs(root_dir):
    pattern = re.compile(r"compute_log_\d{4}-\d{2}-\d{2}_\d{2}[:\-]?\d{2}[:\-]?\d{2}\.csv_top_processes\.csv")
    matching_files = []
    for dirpath, _, filenames in os.walk(root_dir):
        for fname in filenames:
            if pattern.match(fname):
                matching_files.append(os.path.join(dirpath, fname))
    return matching_files

def is_slam_process(proc_name):
    proc_name = proc_name.lower()
    return (
        proc_name.startswith("rtabmap") or
        proc_name.startswith("dlio") or
        proc_name.startswith("icp_odom") or
        proc_name.startswith("rviz") or
        proc_name.startswith("ros")
    )

def process_log_file(file_path):
    try:
        df = pd.read_csv(file_path)
        grouped = df.groupby("name")[["cpu", "ram"]].mean().reset_index()
        grouped = grouped.sort_values(by="cpu", ascending=False)
        return grouped
    except Exception as e:
        print(f"[ERROR] Could not process {file_path}: {e}")
        return None

def write_summary_to_csv(folder, summary_df):
    out_path = os.path.join(folder, "process_usage_summary.csv")
    summary_df.to_csv(out_path, index=False)
    print(f"✔️  Saved summary to: {out_path}")

def summarize_slam_processes(folder_to_summary):
    print("\n📊 SLAM Process Summary (Average CPU and RAM):\n")
    print(f"{'Folder':<25} {'CPU (%)':>10} {'RAM (%)':>10}")
    print("-" * 50)

    for folder, df in folder_to_summary.items():
        slam_df = df[df["name"].apply(is_slam_process)]

        avg_cpu = slam_df["cpu"].sum()
        avg_ram = slam_df["ram"].sum()

        print(f"{os.path.basename(folder):<25} {avg_cpu:>10.2f} {avg_ram:>10.2f}")

def main():
    root_dir = "."
    matching_files = find_matching_logs(root_dir)

    if not matching_files:
        print("❌ No matching compute_log files found.")
        return

    print(f"\n🔍 Found {len(matching_files)} matching files.\n")

    folder_to_summary = {}

    for log_file in matching_files:
        folder = os.path.dirname(log_file)
        print(f"\n📁 Processing folder: {folder}")
        print(f"📄 File: {os.path.basename(log_file)}")

        summary = process_log_file(log_file)
        if summary is not None:
            folder_to_summary[folder] = summary
            print(summary.to_string(index=False))
            write_summary_to_csv(folder, summary)

    summarize_slam_processes(folder_to_summary)

if __name__ == "__main__":
    main()
