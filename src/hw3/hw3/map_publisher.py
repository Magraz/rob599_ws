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

        # Parameters
        self.declare_parameter("ground_truth_topic", "/ground_truth")
        ground_truth_topic = self.get_parameter("ground_truth_topic").value

        # Subscribe to ground truth to initialize map frame
        self.ground_truth_sub = self.create_subscription(
            Odometry, ground_truth_topic, self.ground_truth_callback, 10
        )

        self.get_logger().info(
            "Map Publisher initialized. Waiting for /ground_truth..."
        )

    def ground_truth_callback(self, msg):
        """Initialize map <-> odom transform based on start pose"""
        if not self.map_initialized:
            # map -> odom transform (existing)
            t = TransformStamped()
            t.header.stamp.sec = 0
            t.header.stamp.nanosec = 0
            t.header.frame_id = "map"
            t.child_frame_id = "odom"
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation

            # world -> map identity transform
            t2 = TransformStamped()
            t2.header.stamp.sec = 0
            t2.header.stamp.nanosec = 0
            t2.header.frame_id = "world"
            t2.child_frame_id = "map"
            t2.transform.translation.x = 0.0
            t2.transform.translation.y = 0.0
            t2.transform.translation.z = 0.0
            t2.transform.rotation.x = 0.0
            t2.transform.rotation.y = 0.0
            t2.transform.rotation.z = 0.0
            t2.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform([t, t2])
            self.map_initialized = True

            self.get_logger().info(
                f"Initialized map frame at {t.transform.translation.x:.2f}, {t.transform.translation.y:.2f}"
            )

            self.destroy_subscription(self.ground_truth_sub)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
