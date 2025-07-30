#!/usr/bin/env python3
import os
import signal
import psutil
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import time
from collections import defaultdict

# RTAB-related names (lowercase match)
rtabmap_keywords = ["rtabmap", "icp_odometry", "rtabmap_viz", "rtabmap_slam"]

# To collect data
system_stats = []
process_stats = []
per_process_history = defaultdict(list)

def get_top_processes_by_combined_usage(n=20):
    procs = []
    for p in psutil.process_iter(attrs=['pid', 'name']):
        try:
            with p.oneshot():
                cpu = p.cpu_percent(interval=None)
                mem = p.memory_percent()
                name = p.name()
                if cpu > 0 or mem > 0:
                    procs.append((name, p.pid, cpu, mem))
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    procs.sort(key=lambda x: (x[2] + x[3]), reverse=True)
    return procs[:n]

def monitor(interval=1):
    print("🧪 Monitoring system... Press CTRL+C to stop.")
    start_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"compute_log_{start_time}.csv"

    try:
        while True:
            timestamp = datetime.now().isoformat()
            total_cpu = psutil.cpu_percent(interval=None)
            total_mem = psutil.virtual_memory().percent
            system_stats.append({"timestamp": timestamp, "cpu": total_cpu, "ram": total_mem})

            top_procs = get_top_processes_by_combined_usage()
            for name, pid, cpu, mem in top_procs:
                record = {"timestamp": timestamp, "name": name, "pid": pid, "cpu": cpu, "ram": mem}
                process_stats.append(record)
                per_process_history[(name, pid)].append((timestamp, cpu, mem))

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n🛑 Interrupted. Saving CSV and generating plots...")
        # Save system-level stats
        sys_df = pd.DataFrame(system_stats)
        sys_df.to_csv(f"{filename}_system.csv", index=False)

        # Save top processes
        proc_df = pd.DataFrame(process_stats)
        proc_df.to_csv(f"{filename}_top_processes.csv", index=False)

        # Plot system CPU/RAM usage
        plt.figure()
        sys_df.plot(x='timestamp', y=['cpu', 'ram'], title='System CPU & RAM Usage (%)', figsize=(12, 6))
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.savefig(f"{filename}_system_plot.png")

        # Plot RTAB-specific process usage
        for (name, pid), entries in per_process_history.items():
            if any(k in name.lower() for k in rtabmap_keywords):
                ts, cpu_vals, ram_vals = zip(*entries)
                df = pd.DataFrame({"timestamp": ts, "cpu": cpu_vals, "ram": ram_vals})
                df.plot(x='timestamp', y=['cpu', 'ram'], title=f"{name} (PID {pid}) Usage", figsize=(12, 6))
                plt.xticks(rotation=45)
                plt.tight_layout()
                plt.savefig(f"{filename}_{name}_{pid}.png")

        print("✅ Done! CSV and plots saved.")
        print(f"📁 Files saved with base name: {filename}")

if __name__ == "__main__":
    # Warm-up to get valid cpu_percent (first call returns 0)
    for p in psutil.process_iter():
        try:
            p.cpu_percent(interval=None)
        except Exception:
            continue
    monitor(interval=1)
