import os
import csv

def parse_cpu_percent(value):
    try:
        return float(value.strip().replace('%', ''))
    except:
        return 0.0

def extract_cpu_values(csv_path):
    total_cpu = 0.0
    slam_viz_cpu = 0.0
    slam_only_cpu = 0.0

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            total_cpu = float(row["CPU (%)"])
            per_process_cpu = row["Per Process CPU (%)"]

            slam_viz_cpu = 0.0
            slam_only_cpu = 0.0

            for item in per_process_cpu.split(','):
                if ':' not in item:
                    continue
                name, percent = item.strip().split(':')
                name = name.strip()
                percent = parse_cpu_percent(percent)

                if name in ['rtabmap_viz', 'rviz2']:
                    slam_viz_cpu += percent
                else:
                    slam_only_cpu += percent

            break  # Only process the first row

    return total_cpu, slam_viz_cpu, slam_only_cpu

def scan_folders(root="."):
    print(f"{'Folder':<20} {'Total CPU (%)':>15} {'SLAM+Viz (% of 8-core)':>25} {'SLAM-only (% of 8-core)':>30}")
    print("-" * 95)

    for dirpath, _, filenames in os.walk(root):
        if "compute_footprint_summary.csv" in filenames:
            csv_path = os.path.join(dirpath, "compute_footprint_summary.csv")
            folder_name = os.path.basename(dirpath)
            try:
                total_cpu, slam_viz, slam_only = extract_cpu_values(csv_path)
                slam_viz_norm = round(slam_viz / 800 * 100, 2)  # 800 = 8 cores * 100%
                slam_only_norm = round(slam_only / 800 * 100, 2)
                print(f"{folder_name:<20} {total_cpu:>15.2f} {slam_viz_norm:>25.2f} {slam_only_norm:>30.2f}")
            except Exception as e:
                print(f"{folder_name:<20} ERROR reading file: {e}")

if __name__ == "__main__":
    scan_folders(".")
