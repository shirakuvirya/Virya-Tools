#!/usr/bin/env python3
"""
SLAM Benchmark Logger — Passive System Monitor

Launch this **before** your SLAM process (e.g. Cartographer+RViz).
It will:
  - Monitor CPU, RAM, Disk, Network, Temperature, Load, Swap, Per-process RAM
  - Detect when `rviz2` or `cartographer_node` starts
  - Log performance data every second
  - Save graphs and metadata to a timestamped folder

Usage:
  $ python3 slam_benchmark_logger.py
"""

import os
import psutil
import time
import datetime
import threading
import subprocess
import matplotlib.pyplot as plt
from pathlib import Path

# ─────────────────────────── Setup Folder ─────────────────────────────
timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
run_dir = Path(f"slam_benchmark_runs/{timestamp}")
run_dir.mkdir(parents=True, exist_ok=True)
log_path = run_dir / "resource_usage.csv"
meta_path = run_dir / "run_metadata.txt"

print(f"📦 Logging to {run_dir}")

# ──────────────────────── Process Detection ───────────────────────────
def find_target_pids():
    targets = ["rviz2", "cartographer_node"]
    pids = []
    for proc in psutil.process_iter(['pid', 'name']):
        if any(name in proc.info['name'] for name in targets):
            pids.append(proc.info['pid'])
    return pids

# ──────────────────────── Logging Function ────────────────────────────
usage_data = []
def monitor():
    net_io_start = psutil.net_io_counters()
    bag_size_start = get_bag_size()
    with open(log_path, "w") as f:
        f.write("time_sec,cpu_percent,mem_used_mb,disk_used_mb,net_sent_mb,net_recv_mb,temp_c,load_1m,swap_used_mb,rviz_mem_mb,carto_mem_mb,bag_size_mb\n")
        t0 = time.time()
        try:
            while True:
                time.sleep(1)
                now = time.time()
                cpu = psutil.cpu_percent()
                mem = psutil.virtual_memory().used / (1024**2)
                disk = psutil.disk_usage('/').used / (1024**2)
                net_io = psutil.net_io_counters()
                net_sent = (net_io.bytes_sent - net_io_start.bytes_sent) / (1024**2)
                net_recv = (net_io.bytes_recv - net_io_start.bytes_recv) / (1024**2)

                temp = get_cpu_temp()
                load = os.getloadavg()[0]
                swap = psutil.swap_memory().used / (1024**2)

                rviz_mem = get_proc_mem("rviz2")
                carto_mem = get_proc_mem("cartographer_node")
                bag_size = get_bag_size() - bag_size_start

                f.write(f"{now - t0:.1f},{cpu:.2f},{mem:.2f},{disk:.2f},{net_sent:.2f},{net_recv:.2f},{temp:.1f},{load:.2f},{swap:.2f},{rviz_mem:.2f},{carto_mem:.2f},{bag_size:.2f}\n")
                usage_data.append((now - t0, cpu, mem, disk, net_sent, net_recv, temp, load, swap, rviz_mem, carto_mem, bag_size))
        except KeyboardInterrupt:
            print("🛑 Logging stopped.")

# ───────────────────────── Helper Functions ──────────────────────────
def get_cpu_temp():
    try:
        temps = psutil.sensors_temperatures()
        for name in temps:
            for entry in temps[name]:
                if hasattr(entry, 'current'):
                    return entry.current
    except Exception:
        return -1.0

def get_proc_mem(proc_name):
    for proc in psutil.process_iter(['name', 'memory_info']):
        if proc.info['name'] == proc_name:
            return proc.info['memory_info'].rss / (1024**2)
    return 0.0

def get_bag_size():
    total = 0
    bag_dir = Path.home() / "Downloads" / "bag_files"
    for path in bag_dir.glob("**/*.db3"):
        total += path.stat().st_size
    return total / (1024**2)

# ───────────────────────── Graphing Function ──────────────────────────
def generate_graphs():
    (times, cpus, mems, disks, netsent, netrecv, temps, loads, swaps, rvizs, cartos, bags) = zip(*usage_data)

    plt.figure()
    plt.plot(times, cpus, label="CPU %")
    plt.xlabel("Time (s)")
    plt.ylabel("CPU Usage (%)")
    plt.grid(True)
    plt.savefig(run_dir / "cpu_usage.png")

    plt.figure()
    plt.plot(times, mems, label="RAM (MB)")
    plt.xlabel("Time (s)")
    plt.ylabel("Memory Usage (MB)")
    plt.grid(True)
    plt.savefig(run_dir / "memory_usage.png")

    plt.figure()
    plt.plot(times, disks, label="Disk Used (MB)")
    plt.xlabel("Time (s)")
    plt.ylabel("Disk Usage (MB)")
    plt.grid(True)
    plt.savefig(run_dir / "disk_usage.png")

    plt.figure()
    plt.plot(times, netsent, label="Net Sent (MB)")
    plt.plot(times, netrecv, label="Net Recv (MB)")
    plt.xlabel("Time (s)")
    plt.ylabel("Network (MB)")
    plt.grid(True)
    plt.legend()
    plt.savefig(run_dir / "network_usage.png")

    plt.figure()
    plt.plot(times, temps, label="CPU Temp (°C)")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (°C)")
    plt.grid(True)
    plt.savefig(run_dir / "temperature.png")

    plt.figure()
    plt.plot(times, loads, label="System Load 1m")
    plt.xlabel("Time (s)")
    plt.ylabel("Load Average")
    plt.grid(True)
    plt.savefig(run_dir / "load_average.png")

    plt.figure()
    plt.plot(times, rvizs, label="RViz RAM (MB)")
    plt.plot(times, cartos, label="Cartographer RAM (MB)")
    plt.xlabel("Time (s)")
    plt.ylabel("Process RAM Usage (MB)")
    plt.legend()
    plt.grid(True)
    plt.savefig(run_dir / "process_memory.png")

    plt.figure()
    plt.plot(times, bags, label="Bag Size Growth (MB)")
    plt.xlabel("Time (s)")
    plt.ylabel("Bag File Size Δ (MB)")
    plt.grid(True)
    plt.savefig(run_dir / "bag_size.png")

# ──────────────────────────── Run Monitor ─────────────────────────────
print("⌛ Waiting for SLAM processes to start...")
while not find_target_pids():
    time.sleep(1)
print("✅ Detected SLAM activity. Starting monitoring...")

# Start monitoring in main thread
start_time = time.time()
monitor()
end_time = time.time()

# ───────────────────────────── Metadata ───────────────────────────────
with open(meta_path, "w") as meta:
    meta.write(f"Start Time: {datetime.datetime.fromtimestamp(start_time)}\n")
    meta.write(f"End Time: {datetime.datetime.fromtimestamp(end_time)}\n")
    meta.write(f"Duration: {end_time - start_time:.2f} seconds\n")
    meta.write(f"User: {os.getlogin()}\n")
    meta.write(f"Run Directory: {run_dir.resolve()}\n")
    meta.write(f"Detected SLAM PIDs: {find_target_pids()}\n")

# ───────────────────────────── Wrap Up ────────────────────────────────
print("📊 Generating performance graphs...")
generate_graphs()
print(f"✅ All logs saved to: {run_dir.resolve()}")
