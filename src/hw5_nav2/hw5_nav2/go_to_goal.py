"""Send the robot to a fixed goal using the Nav2 NavigateToPose action."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class GoToGoal(Node):

    def __init__(self):
        super().__init__("go_to_goal")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self._marker_pub = self.create_publisher(Marker, "goal_marker", 1)
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Waiting for navigate_to_pose action server...")
        self._client.wait_for_server()
        self.get_logger().info("Action server available.")

    def send_goal(self, x: float, y: float):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal: ({x:.2f}, {y:.2f})")
        self._publish_marker(x, y)

        self._send_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        self._send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted.")
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        pos = feedback_msg.feedback.current_pose.pose.position
        self.get_logger().info(
            f"Current position: ({pos.x:.2f}, {pos.y:.2f})",
            throttle_duration_sec=2.0,
        )

    def _publish_marker(self, x: float, y: float):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "goal"
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.15
        m.pose.orientation.w = 1.0
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.3
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.8
        self._marker_pub.publish(m)

    def _result_cb(self, future):
        result = future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info("Goal reached!")
        else:
            self.get_logger().warn(f"Navigation finished with status: {result.status}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    x = node.get_parameter("x").get_parameter_value().double_value
    y = node.get_parameter("y").get_parameter_value().double_value
    node.send_goal(x, y)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
