#!/usr/bin/env python3

import os
import pandas as pd

def summarize_loop_errors(root_dir="."):
    summaries = []

    for dirpath, _, filenames in os.walk(root_dir):
        if "closest_loop_error.csv" in filenames:
            path = os.path.join(dirpath, "closest_loop_error.csv")
            try:
                df = pd.read_csv(path)
                if len(df) == 0:
                    continue
                mean_dx = df["dx"].mean()
                mean_dy = df["dy"].mean()
                mean_dz = df["dz"].mean()
                mean_euclidean = df["euclidean"].mean()
                mean_rmse = df["rmse"].mean()
                summaries.append({
                    "folder": os.path.relpath(dirpath, root_dir),
                    "mean_dx": mean_dx,
                    "mean_dy": mean_dy,
                    "mean_dz": mean_dz,
                    "mean_euclidean": mean_euclidean,
                    "mean_rmse": mean_rmse,
                    "num_loops": len(df)
                })
            except Exception as e:
                print(f"⚠️  Failed to read {path}: {e}")

    if not summaries:
        print("❌ No valid 'closest_loop_error.csv' files found.")
        return

    summary_df = pd.DataFrame(summaries)
    output_path = os.path.join(root_dir, "loop_error_summary.csv")
    summary_df.to_csv(output_path, index=False)
    print(f"\n✅ Summary written to: {output_path}")
    print(summary_df)

if __name__ == "__main__":
    summarize_loop_errors(".")
