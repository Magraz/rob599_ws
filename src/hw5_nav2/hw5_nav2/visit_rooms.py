"""Visit each room in the house, spinning in place at each one."""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose, Spin
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

ROOMS = [
    (3.70, -1.36),
    (6.78, 3.03),
    (3.18, 3.71),
    (7.7, -0.70),
    (-0.43, 4.30),
    (-3.82, 3.82),
    (-4.38, -1.00),
]


class VisitRooms(Node):

    def __init__(self):
        super().__init__("visit_rooms")
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._spin_client = ActionClient(self, Spin, "spin")
        self._marker_pub = self.create_publisher(MarkerArray, "room_markers", 1)

        self.get_logger().info("Waiting for action servers...")
        self._nav_client.wait_for_server()
        self._spin_client.wait_for_server()
        self.get_logger().info("Action servers available.")

        self._room_index = 0
        self._visited = set()
        self._publish_all_markers()
        self._go_to_next_room()

    # ---- navigation ----

    def _go_to_next_room(self):
        if self._room_index >= len(ROOMS):
            self.get_logger().info("All rooms visited!")
            rclpy.shutdown()
            return

        x, y = ROOMS[self._room_index]
        self.get_logger().info(
            f"Room {self._room_index + 1}/{len(ROOMS)}: going to ({x:.2f}, {y:.2f})"
        )

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_cb
        )
        future.add_done_callback(self._nav_response_cb)

    def _nav_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected, skipping room.")
            self._room_index += 1
            self._go_to_next_room()
            return

        self.get_logger().info("Navigation goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg):
        pos = feedback_msg.feedback.current_pose.pose.position
        self.get_logger().info(
            f"Position: ({pos.x:.2f}, {pos.y:.2f})",
            throttle_duration_sec=3.0,
        )

    def _nav_result_cb(self, future):
        result = future.result()
        if result.status == 4:
            self._visited.add(self._room_index)
            self._publish_all_markers()
            self.get_logger().info("Arrived at room. Spinning...")
            self._do_spin()
        else:
            self.get_logger().warn(
                f"Navigation ended with status {result.status}, moving on."
            )
            self._room_index += 1
            self._go_to_next_room()

    # ---- spin ----

    def _do_spin(self):
        goal = Spin.Goal()
        goal.target_yaw = 2.0 * math.pi
        future = self._spin_client.send_goal_async(goal)
        future.add_done_callback(self._spin_response_cb)

    def _spin_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Spin rejected, moving on.")
            self._room_index += 1
            self._go_to_next_room()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._spin_result_cb)

    def _spin_result_cb(self, future):
        self.get_logger().info("Spin complete.")
        self._room_index += 1
        self._go_to_next_room()

    # ---- markers ----

    def _publish_all_markers(self):
        ma = MarkerArray()
        for i, (x, y) in enumerate(ROOMS):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "rooms"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            visited = i in self._visited
            m.color.r = 0.0 if visited else 1.0
            m.color.g = 1.0 if visited else 0.0
            m.color.b = 0.0
            m.color.a = 0.8
            ma.markers.append(m)
        self._marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = VisitRooms()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
