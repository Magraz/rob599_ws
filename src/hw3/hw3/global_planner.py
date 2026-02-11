#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import numpy as np


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__("global_planner")

        # Start position (world coordinates)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)

        # Goal position (world coordinates)
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)

        # Store latest map
        self.latest_map: OccupancyGrid | None = None
        self.map_data: np.ndarray | None = None
        self.map_info = None

        # Subscribe to the map
        self.sub_map = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 1
        )

        # Service to trigger planning
        self.srv_plan = self.create_service(
            Trigger, "plan_path", self.plan_path_callback
        )

        # Publisher for the planned path
        self.pub_path = self.create_publisher(Path, "/planned_path", 1)

        self.get_logger().info(
            "Path Planner ready. Call 'plan_path' service to compute a path."
        )

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

    def plan_path_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self.latest_map is None or self.map_data is None:
            response.success = False
            response.message = "No map received yet."
            self.get_logger().warn("Plan requested but no map has been received.")
            return response

        # Read start and goal from parameters (allows dynamic reconfigure)
        start_x = self.get_parameter("start_x").value
        start_y = self.get_parameter("start_y").value
        goal_x = self.get_parameter("goal_x").value
        goal_y = self.get_parameter("goal_y").value

        self.get_logger().info(
            f"Planning from ({start_x}, {start_y}) to ({goal_x}, {goal_y})"
        )

        # Convert world coordinates to grid indices
        start_cell = self.world_to_grid(start_x, start_y)
        goal_cell = self.world_to_grid(goal_x, goal_y)

        # Validate that cells are within bounds
        if not self.is_valid_cell(start_cell):
            response.success = False
            response.message = (
                f"Start position ({start_x}, {start_y}) is outside the map."
            )
            self.get_logger().error(response.message)
            return response

        if not self.is_valid_cell(goal_cell):
            response.success = False
            response.message = f"Goal position ({goal_x}, {goal_y}) is outside the map."
            self.get_logger().error(response.message)
            return response

        # TODO: Implement your planning algorithm here
        # Input:
        #   self.map_data  — 2D numpy array (height x width), values: -1=unknown, 0=free, 100=occupied
        #   start_cell     — (col, row) tuple in grid coordinates
        #   goal_cell      — (col, row) tuple in grid coordinates
        #
        # Output:
        #   path_cells     — list of (col, row) tuples from start to goal, or None if no path found

        path_cells = self.compute_path(start_cell, goal_cell)

        if path_cells is None or len(path_cells) == 0:
            response.success = False
            response.message = "No path found."
            self.get_logger().warn(response.message)
            return response

        # Convert grid path to ROS Path message and publish
        path_msg = self.cells_to_path(path_cells)
        self.pub_path.publish(path_msg)

        response.success = True
        response.message = f"Path found with {len(path_cells)} waypoints."
        self.get_logger().info(response.message)
        return response

    def compute_path(
        self, start: tuple[int, int], goal: tuple[int, int]
    ) -> list[tuple[int, int]] | None:
        """Compute a path from start to goal on the grid.

        Args:
            start: (col, row) in grid coordinates.
            goal: (col, row) in grid coordinates.

        Returns:
            A list of (col, row) tuples representing the path, or None if no path exists.
        """
        # TODO: Implement planning algorithm (A*, Dijkstra, RRT, etc.)
        self.get_logger().warn("compute_path() is not yet implemented!")
        return None

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert world coordinates to grid cell (col, row)."""
        assert self.map_info is not None
        col = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return col, row

    def grid_to_world(self, col: int, row: int) -> tuple[float, float]:
        """Convert grid cell (col, row) to world coordinates (center of cell)."""
        assert self.map_info is not None
        x = self.map_info.origin.position.x + (col + 0.5) * self.map_info.resolution
        y = self.map_info.origin.position.y + (row + 0.5) * self.map_info.resolution
        return x, y

    def is_valid_cell(self, cell: tuple[int, int]) -> bool:
        """Check if a grid cell is within map bounds."""
        assert self.map_info is not None
        col, row = cell
        return 0 <= col < self.map_info.width and 0 <= row < self.map_info.height

    def cells_to_path(self, cells: list[tuple[int, int]]) -> Path:
        """Convert a list of grid cells to a ROS Path message."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"

        for col, row in cells:
            wx, wy = self.grid_to_world(col, row)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
