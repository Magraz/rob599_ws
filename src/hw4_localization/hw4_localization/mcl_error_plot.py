#!/usr/bin/env python3
"""
Live-plotting node that shows MCL localization error over time.

Subscribes to the MCL estimated pose and the Stage ground truth,
computes position and heading error, and displays a live matplotlib plot.

Usage:
    ros2 run hw4_localization mcl_error_plot
    ros2 run hw4_localization mcl_error_plot --ros-args -p mcl_pose_topic:=/mcl/pose
"""

import math
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import matplotlib.animation as animation


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    """Signed shortest angular difference, wrapped to [-pi, pi]."""
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class MCLErrorPlot(Node):
    def __init__(self):
        super().__init__("mcl_error_plot")

        self.declare_parameter("mcl_pose_topic", "/mcl/pose")
        self.declare_parameter("ground_truth_topic", "/ground_truth")

        mcl_topic = self.get_parameter("mcl_pose_topic").value
        gt_topic = self.get_parameter("ground_truth_topic").value

        # Latest ground truth pose
        self.gt_x = None
        self.gt_y = None
        self.gt_yaw = None
        self.lock = threading.Lock()

        # Error history
        self.times = []
        self.pos_errors = []
        self.yaw_errors = []
        self.t0 = None

        self.create_subscription(Odometry, gt_topic, self._gt_cb, 10)
        self.create_subscription(PoseStamped, mcl_topic, self._mcl_cb, 10)

        self.get_logger().info(
            f"Error plot: mcl={mcl_topic}, gt={gt_topic}"
        )

    def _gt_cb(self, msg: Odometry):
        with self.lock:
            self.gt_x = msg.pose.pose.position.x
            self.gt_y = msg.pose.pose.position.y
            self.gt_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _mcl_cb(self, msg: PoseStamped):
        with self.lock:
            if self.gt_x is None:
                return

            est_x = msg.pose.position.x
            est_y = msg.pose.position.y
            est_yaw = yaw_from_quaternion(msg.pose.orientation)

            pos_err = math.hypot(est_x - self.gt_x, est_y - self.gt_y)
            yaw_err = abs(angle_diff(est_yaw, self.gt_yaw))

            now = self.get_clock().now().nanoseconds * 1e-9
            if self.t0 is None:
                self.t0 = now
            t = now - self.t0

            self.times.append(t)
            self.pos_errors.append(pos_err)
            self.yaw_errors.append(math.degrees(yaw_err))


def main(args=None):
    rclpy.init(args=args)
    node = MCLErrorPlot()

    # Spin ROS in a background thread so matplotlib can own the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Set up live plot
    fig, (ax_pos, ax_yaw) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    fig.suptitle("MCL Localization Error")

    (line_pos,) = ax_pos.plot([], [], "b-", linewidth=1.5)
    ax_pos.set_ylabel("Position error (m)")
    ax_pos.set_ylim(0, 2.0)
    ax_pos.grid(True, alpha=0.3)

    (line_yaw,) = ax_yaw.plot([], [], "r-", linewidth=1.5)
    ax_yaw.set_ylabel("Heading error (deg)")
    ax_yaw.set_xlabel("Time (s)")
    ax_yaw.set_ylim(0, 90.0)
    ax_yaw.grid(True, alpha=0.3)

    def update(_frame):
        with node.lock:
            if not node.times:
                return line_pos, line_yaw

            t = list(node.times)
            pe = list(node.pos_errors)
            ye = list(node.yaw_errors)

        line_pos.set_data(t, pe)
        line_yaw.set_data(t, ye)

        # Auto-scale axes
        if t:
            ax_pos.set_xlim(0, max(t[-1], 1.0))
            ax_yaw.set_xlim(0, max(t[-1], 1.0))
            ax_pos.set_ylim(0, max(max(pe) * 1.2, 0.1))
            ax_yaw.set_ylim(0, max(max(ye) * 1.2, 1.0))

        return line_pos, line_yaw

    _ani = animation.FuncAnimation(fig, update, interval=200, blit=False)
    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
