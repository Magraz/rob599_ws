#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster


class MapPublisher(Node):
    def __init__(self):
        super().__init__("map_publisher")

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.map_initialized = False

        # Subscribe to ground truth to initialize map frame
        self.ground_truth_sub = self.create_subscription(
            Odometry, "/ground_truth", self.ground_truth_callback, 10
        )

        self.get_logger().info(
            "Map Publisher initialized. Waiting for /ground_truth..."
        )

    def ground_truth_callback(self, msg):
        """Initialize map <-> odom transform based on start pose"""
        if not self.map_initialized:
            t = TransformStamped()

            # Use current time
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = "odom"

            # The transform from map to odom is effectively the initial ground truth pose
            # because odom starts at (0,0) relative to its own frame.
            # So, T_map_odom = Pose_robot_in_map_at_start
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation

            self.tf_broadcaster.sendTransform(t)
            self.map_initialized = True

            self.get_logger().info(
                f"Initialized map frame at {t.transform.translation.x:.2f}, {t.transform.translation.y:.2f}"
            )

            # Unsubscribe after initialization if you only want it static
            self.destroy_subscription(self.ground_truth_sub)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
