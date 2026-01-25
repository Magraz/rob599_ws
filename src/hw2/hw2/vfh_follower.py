#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class VFHFollower(Node):
    def __init__(self):
        super().__init__("vfh_follower")

        # Parameters
        self.declare_parameter("linear_speed", 0.4)
        self.declare_parameter("angular_speed_max", 1.0)
        self.declare_parameter("goal_tolerance", 0.3)
        self.declare_parameter("safety_radius", 2.0)
        self.declare_parameter("sector_count", 72)
        self.declare_parameter("robot_radius", 0.5)

        self.robot_radius = self.get_parameter("robot_radius").value
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

        self.get_logger().info("VFH Follower ready. Call /start_follower to begin.")

    def start_follower_callback(self, request, response):
        self.active = request.data
        response.success = True
        response.message = (
            "VFH follower started." if self.active else "VFH follower stopped."
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

        target_pose = self.waypoints[self.current_waypoint_index]
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        dist_to_goal = math.sqrt(dx**2 + dy**2)

        if dist_to_goal < self.goal_tolerance:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.finished = True
                self.stop_robot()
                self.get_logger().info("All waypoints finished.")
                return
            target_pose = self.waypoints[self.current_waypoint_index]
            dx = target_pose.position.x - self.current_pose.position.x
            dy = target_pose.position.y - self.current_pose.position.y

        curr_yaw = get_yaw_from_quaternion(self.current_pose.orientation)
        target_heading = math.atan2(dy, dx)
        desired_yaw = normalize_angle(target_heading - curr_yaw)

        histogram = [0.0] * self.sector_count
        sector_size = (2 * math.pi) / self.sector_count

        angle = self.current_scan.angle_min
        inc = self.current_scan.angle_increment

        for r in self.current_scan.ranges:
            if math.isinf(r) or math.isnan(r):
                r = self.current_scan.range_max
            if r < self.safety_radius:
                norm_angle = normalize_angle(angle)
                sector_idx = int((norm_angle + math.pi) / sector_size)
                sector_idx = max(0, min(sector_idx, self.sector_count - 1))

                # Inflate by robot size
                inflate_angle = math.atan2(self.robot_radius, max(r, 0.05))
                inflate_sectors = int(math.ceil(inflate_angle / sector_size))

                for k in range(-inflate_sectors, inflate_sectors + 1):
                    histogram[(sector_idx + k) % self.sector_count] += 1

            angle += inc

        goal_sector = int((desired_yaw + math.pi) / sector_size)
        goal_sector = max(0, min(goal_sector, self.sector_count - 1))

        best_sector = -1
        found_safe = False

        for offset in range(self.sector_count):
            if offset == 0:
                indices = [goal_sector]
            else:
                indices = [
                    (goal_sector + offset) % self.sector_count,
                    (goal_sector - offset) % self.sector_count,
                ]
            for idx in indices:
                if histogram[idx] == 0:
                    best_sector = idx
                    found_safe = True
                    break
            if found_safe:
                break

        twist = Twist()
        if found_safe:
            steer_angle = (best_sector * sector_size) - math.pi + (sector_size / 2.0)
            twist.angular.z = max(
                min(steer_angle * 2.0, self.angular_speed_max),
                -self.angular_speed_max,
            )
            twist.linear.x = self.linear_speed * (
                0.2 if abs(steer_angle) > 0.5 else 1.0
            )
        else:
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed_max

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
