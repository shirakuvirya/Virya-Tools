#!/usr/bin/env python3

import os
import csv
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

# Configurable thresholds
SPATIAL_RADIUS = 0.15       # max radius to consider for neighbor search
MIN_FRAME_DIFF = 20        # skip immediate neighbors by frame index
MIN_TIME_DIFF = 5.0        # seconds — skip close-in-time matches

def read_trajectory(csv_path):
    timestamps = []
    positions = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                timestamps.append(float(row["timestamp"]))
                positions.append([
                    float(row["x"]),
                    float(row["y"]),
                    float(row["z"])
                ])
            except Exception as e:
                print(f"⚠️  Skipping row in {csv_path} due to error: {e}")
    return np.array(timestamps), np.array(positions)

def find_closest_loop_errors(timestamps, positions):
    tree = cKDTree(positions)
    results = []

    for i, (pos_i, t_i) in enumerate(zip(positions, timestamps)):
        dists, idxs = tree.query(pos_i, k=10, distance_upper_bound=SPATIAL_RADIUS)

        best_j = -1
        best_dist = float('inf')

        if not np.isscalar(dists):
            for dist, j in zip(dists, idxs):
                if j >= len(positions) or j == i:
                    continue
                if abs(j - i) < MIN_FRAME_DIFF:
                    continue
                if abs(timestamps[j] - t_i) < MIN_TIME_DIFF:
                    continue
                if dist < best_dist:
                    best_dist = dist
                    best_j = j

        if best_j != -1 and best_dist != float('inf'):
            error_vector = positions[i] - positions[best_j]
            rmse = np.sqrt(np.mean(error_vector**2))
            results.append({
                "i": i,
                "j": best_j,
                "t_i": t_i,
                "t_j": timestamps[best_j],
                "dt": abs(timestamps[best_j] - t_i),
                "dx": error_vector[0],
                "dy": error_vector[1],
                "dz": error_vector[2],
                "euclidean": best_dist,
                "rmse": rmse
            })

    return results

def save_results(results, path):
    if not results:
        print(f"ℹ️  No valid loop candidates found for: {path}")
        return
    df = pd.DataFrame(results)
    df.to_csv(path, index=False)
    print(f"✅ Saved loop error summary: {path}")

def process_csv(csv_path):
    print(f"\n🔍 Processing: {csv_path}")
    timestamps, positions = read_trajectory(csv_path)
    if len(timestamps) < 50:
        print("⚠️  Too few odometry points, skipping.")
        return
    results = find_closest_loop_errors(timestamps, positions)
    output_path = os.path.join(os.path.dirname(csv_path), "closest_loop_error.csv")
    save_results(results, output_path)

def recursive_loop_check(root_dir="."):
    count = 0
    for dirpath, _, filenames in os.walk(root_dir):
        if "odom_from_mcap.csv" in filenames:
            csv_path = os.path.join(dirpath, "odom_from_mcap.csv")
            process_csv(csv_path)
            count += 1
    if count == 0:
        print("❌ No odom_from_mcap.csv files found.")
    else:
        print(f"\n🏁 Finished processing {count} trajectory files.")

if __name__ == "__main__":
    recursive_loop_check(".")

