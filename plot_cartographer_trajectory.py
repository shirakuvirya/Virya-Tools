#!/usr/bin/env python3
"""
Query Cartographer's /trajectory_query service and plot the 3-D trajectory
plus z-axis over time.

usage:
    python3 plot_cartographer_trajectory.py            # defaults to trajectory 0
    python3 plot_cartographer_trajectory.py 2          # pick another trajectory id
"""

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import TrajectoryQuery
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D            # noqa: F401 – just activates 3-D projection


class TrajectoryPlotter(Node):
    def __init__(self, traj_id: int):
        super().__init__('trajectory_plotter')
        self._traj_id = traj_id

        self._cli = self.create_client(TrajectoryQuery, '/trajectory_query')
        self.get_logger().info('Waiting for /trajectory_query service…')
        self._cli.wait_for_service()
        self.get_logger().info('Service available, sending request.')

        req = TrajectoryQuery.Request()
        req.trajectory_id = self._traj_id

        future = self._cli.call_async(req)
        future.add_done_callback(self._got_response)

    # ---------- callbacks -------------------------------------------------

    def _got_response(self, future):
        res = future.result()
        if res.status.code != 0:
            self.get_logger().error(f'Cartographer returned an error: {res.status.message}')
            rclpy.shutdown()
            return

        # ---- unpack into numpy arrays ------------------------------------
        times  = []
        xyz    = []

        for stamped in res.trajectory:
            t = stamped.header.stamp.sec + stamped.header.stamp.nanosec * 1e-9
            p = stamped.pose.position
            times.append(t)
            xyz.append([p.x, p.y, p.z])

        times = np.asarray(times) - times[0]          # start time = 0 s
        xyz   = np.asarray(xyz)

        self._make_plots(times, xyz)
        rclpy.shutdown()

    # ---------- plotting ---------------------------------------------------

    @staticmethod
    def _make_plots(t, xyz):
        fig = plt.figure(figsize=(12, 5))

        # 3-D trajectory
        ax = fig.add_subplot(1, 2, 1, projection='3d')
        ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], lw=1)
        ax.set_title('Trajectory in map frame')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('z [m]')
        ax.view_init(elev=30, azim=-120)              # nice default view

        # z vs. time
        ax2 = fig.add_subplot(1, 2, 2)
        ax2.plot(t, xyz[:, 2])
        ax2.set_title('z-axis over time')
        ax2.set_xlabel('time [s]')
        ax2.set_ylabel('z [m]')

        plt.tight_layout()
        plt.show()



# ---------- main ----------------------------------------------------------
def main():
    traj_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    rclpy.init()

    node = TrajectoryPlotter(traj_id)      # keep a reference
    try:
        rclpy.spin(node)                   # <-- spin the *node*
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

