#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
import math


def get_yaw_from_quaternion(q):
    """Convert orientation quaternion to Euler yaw angle."""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class VFHFollower(Node):
    def __init__(self):
        super().__init__("vfh_follower")

        # Parameters
        self.declare_parameter("linear_speed", 0.5)
        self.declare_parameter("angular_speed_max", 1.0)
        self.declare_parameter("goal_tolerance", 0.3)
        self.declare_parameter("safety_radius", 1.0)  # How close obstacles can be
        self.declare_parameter("sector_count", 72)  # 360 / 5 degree resolution

        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed_max = self.get_parameter("angular_speed_max").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.safety_radius = self.get_parameter("safety_radius").value
        self.sector_count = self.get_parameter("sector_count").value

        # State
        self.waypoints = []
        self.current_waypoint_index = 0
        self.current_pose = None
        self.current_scan = None
        self.active = False
        self.goal_received = False
        self.finished = False

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "/ground_truth", self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/base_scan", self.scan_callback, 10
        )
        self.waypoints_sub = self.create_subscription(
            PoseArray, "/waypoints", self.waypoints_callback, 10
        )

        # Service
        self.srv = self.create_service(
            SetBool, "start_vfh_follower", self.start_follower_callback
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Control Loop
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("VFH Follower Initialized. Waiting for start service...")

    def start_follower_callback(self, request, response):
        self.active = request.data
        response.success = True
        response.message = (
            "VFH Follower Active" if self.active else "VFH Follower Stopped"
        )
        if not self.active:
            self.stop_robot()
        self.get_logger().info(response.message)
        return response

    def waypoints_callback(self, msg):
        if not self.goal_received:
            self.waypoints = msg.poses
            self.current_waypoint_index = 0
            self.goal_received = True
            self.finished = False
            self.get_logger().info(f"Received {len(self.waypoints)} waypoints.")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        self.current_scan = msg

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def control_loop(self):
        if not self.active or not self.goal_received or self.finished:
            return
        if self.current_pose is None or self.current_scan is None:
            return

        # 1. Check Goal Status
        target_pose = self.waypoints[self.current_waypoint_index]
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        dist_to_goal = math.sqrt(dx**2 + dy**2)

        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached.")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.finished = True
                self.stop_robot()
                self.get_logger().info("All waypoints finished!")
                return
            # Update target for next iteration (recalc dx/dy not strictly needed till next loop but good for clarity)
            target_pose = self.waypoints[self.current_waypoint_index]
            dx = target_pose.position.x - self.current_pose.position.x
            dy = target_pose.position.y - self.current_pose.position.y

        # 2. Desired Heading to Goal
        curr_yaw = get_yaw_from_quaternion(self.current_pose.orientation)
        target_heading = math.atan2(dy, dx)
        desired_yaw = normalize_angle(target_heading - curr_yaw)  # Relative to robot

        # 3. Build Polar Histogram (Simplified VFH)
        # Sectors cover [-pi, pi].
        histogram = [0.0] * self.sector_count
        sector_size = (2 * math.pi) / self.sector_count

        # Scan data is usually -135 to +135 or similar. Map ranges to sectors.
        angle = self.current_scan.angle_min
        inc = self.current_scan.angle_increment

        for r in self.current_scan.ranges:
            if math.isinf(r) or math.isnan(r):
                r = self.current_scan.range_max

            # If obstacle is close, mark sector as magnitude (1/dist squared or similar)
            # Here: Binary threshold for simplicity. If close obstacle => High Value
            if r < self.safety_radius:
                # Wrap angle to [-pi, pi]
                norm_angle = normalize_angle(angle)
                # Find sector index (0 to sector_count-1)
                # Map -pi..pi to 0..sector_count
                sector_idx = int((norm_angle + math.pi) / sector_size)
                sector_idx = max(0, min(sector_idx, self.sector_count - 1))

                # Add density (simple counter/binary)
                histogram[sector_idx] += 1

            angle += inc

        # 4. Find Best Steering Direction
        # Convert relative desired yaw to sector index
        goal_sector = int((desired_yaw + math.pi) / sector_size)
        goal_sector = max(0, min(goal_sector, self.sector_count - 1))

        best_sector = -1
        min_cost = float("inf")

        # Heuristic: Cost = k1*|current - target| + k2*|current - wheel_heading| etc.
        # Simple Logic: Search sectors spiral out from goal_sector for a "Valley" (empty bin)

        # We assume 0 in histogram means free. (Value > threshold means blocked)
        # A "safe" sector is one with 0 density (or low density)

        found_safe = False

        # Search array centered on goal_sector
        # Try finding a sector closest to goal_sector that is free
        for offset in range(self.sector_count):
            # Check left and right indices
            indices_to_check = []
            if offset == 0:
                indices_to_check.append(goal_sector)
            else:
                indices_to_check.append((goal_sector + offset) % self.sector_count)
                indices_to_check.append((goal_sector - offset) % self.sector_count)

            for idx in indices_to_check:
                if histogram[idx] == 0:  # Free space
                    best_sector = idx
                    found_safe = True
                    break
            if found_safe:
                break

        # 5. Move Robot
        twist = Twist()

        if found_safe:
            # Convert best sector back to angle relative to robot
            # center of the sector
            steer_angle = (best_sector * sector_size) - math.pi + (sector_size / 2.0)

            # Simple P-control for angle
            twist.angular.z = max(
                min(steer_angle * 2.0, self.angular_speed_max), -self.angular_speed_max
            )

            # Linear speed: slow down if turning sharply
            if abs(steer_angle) > 0.5:
                twist.linear.x = self.linear_speed * 0.2
            else:
                twist.linear.x = self.linear_speed
        else:
            # Blocked everywhere? Rotate in place to find path
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed_max
            self.get_logger().warn("Blocked! Rotating...")

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = VFHFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
