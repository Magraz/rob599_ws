#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, Pose
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class PathToWaypoints(Node):
    def __init__(self):
        super().__init__("path_to_waypoints")

        self.declare_parameter("num_waypoints", 5)
        self.num_waypoints = self.get_parameter("num_waypoints").value

        # Subscribe to the planned path from global planner
        self.path_sub = self.create_subscription(
            Path, "/planned_path", self.path_callback, 1
        )

        # Publisher for PoseArray (consumed by VFHFollower)
        waypoint_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pose_array_pub = self.create_publisher(
            PoseArray, "/waypoints", waypoint_qos
        )

        self.get_logger().info(
            f"PathToWaypoints ready. Will sample {self.num_waypoints} waypoints from /planned_path."
        )

    def path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warn("Received path with fewer than 2 poses, ignoring.")
            return

        # Compute cumulative arc-length along the path
        poses = msg.poses
        cum_dist = [0.0]
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
            dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
            cum_dist.append(cum_dist[-1] + math.hypot(dx, dy))

        total_length = cum_dist[-1]
        if total_length < 1e-6:
            self.get_logger().warn("Path has near-zero length, ignoring.")
            return

        # Sample num_waypoints evenly spaced along the path (excluding start, including goal)
        num_wp = self.num_waypoints
        sampled_poses = []

        for wi in range(num_wp - 1):
            # Target distance along path: evenly spaced, leaving last slot for the goal
            target_dist = total_length * (wi + 1) / num_wp

            # Find the segment that contains this distance
            for j in range(1, len(cum_dist)):
                if cum_dist[j] >= target_dist:
                    # Interpolate between poses[j-1] and poses[j]
                    seg_len = cum_dist[j] - cum_dist[j - 1]
                    if seg_len < 1e-9:
                        t = 0.0
                    else:
                        t = (target_dist - cum_dist[j - 1]) / seg_len

                    p = Pose()
                    p.position.x = poses[j - 1].pose.position.x + t * (
                        poses[j].pose.position.x - poses[j - 1].pose.position.x
                    )
                    p.position.y = poses[j - 1].pose.position.y + t * (
                        poses[j].pose.position.y - poses[j - 1].pose.position.y
                    )
                    p.position.z = 0.0
                    p.orientation.w = 1.0
                    sampled_poses.append(p)
                    break

        # Always include the final goal pose
        goal = Pose()
        goal.position.x = poses[-1].pose.position.x
        goal.position.y = poses[-1].pose.position.y
        goal.position.z = 0.0
        goal.orientation = poses[-1].pose.orientation
        sampled_poses.append(goal)

        # Compute yaw for each waypoint (pointing toward next waypoint)
        for i in range(len(sampled_poses)):
            if i < len(sampled_poses) - 1:
                dx = sampled_poses[i + 1].position.x - sampled_poses[i].position.x
                dy = sampled_poses[i + 1].position.y - sampled_poses[i].position.y
            else:
                # Last waypoint: use direction from previous
                dx = sampled_poses[i].position.x - sampled_poses[i - 1].position.x
                dy = sampled_poses[i].position.y - sampled_poses[i - 1].position.y
            yaw = math.atan2(dy, dx)
            sampled_poses[i].orientation.z = math.sin(yaw / 2.0)
            sampled_poses[i].orientation.w = math.cos(yaw / 2.0)

        # Publish PoseArray
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = msg.header.frame_id
        pose_array.poses = sampled_poses
        self.pose_array_pub.publish(pose_array)

        self.get_logger().info(
            f"Published {len(sampled_poses)} waypoints from path with {len(poses)} poses "
            f"(total length: {total_length:.2f}m)."
        )


def main(args=None):
    rclpy.init(args=args)
    node = PathToWaypoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
