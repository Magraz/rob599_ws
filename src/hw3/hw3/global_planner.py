#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import numpy as np
import math
import heapq
import os
from PIL import Image, ImageDraw
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # needed for do_transform_pose_stamped


class Cell:
    """Represents a cell in the planning grid with A* costs."""

    __slots__ = ("col", "row", "g", "h", "f", "parent")

    def __init__(
        self, col: int, row: int, g: float = float("inf"), h: float = 0.0, parent=None
    ):
        self.col = col
        self.row = row
        self.g = g  # cost from start
        self.h = h  # heuristic cost to goal
        self.f = g + h  # total estimated cost
        self.parent = parent  # parent Cell for path reconstruction

    def update(self, g: float, h: float, parent: "Cell | None" = None):
        """Update costs and parent."""
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    def distance_to(self, other: "Cell") -> float:
        """Calculate the Euclidean distance from this cell to another.

        Args:
            other: The other Cell.

        Returns:
            Euclidean distance between the two cells.
        """
        return math.hypot(other.col - self.col, other.row - self.row)

    def __lt__(self, other: "Cell"):
        return self.f < other.f

    def __eq__(self, other: object):
        if not isinstance(other, Cell):
            return NotImplemented
        return self.col == other.col and self.row == other.row

    def __hash__(self):
        return hash((self.col, self.row))

    def __repr__(self):
        return f"Cell(col={self.col}, row={self.row}, g={self.g:.2f}, h={self.h:.2f}, f={self.f:.2f})"


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__("global_planner")

        # Store latest map
        self.latest_map: OccupancyGrid | None = None
        self.map_data: np.ndarray | None = None
        self.map_info = None
        self.free_mask: np.ndarray | None = None

        # Current robot pose (updated from odometry)
        self.robot_x: float | None = None
        self.robot_y: float | None = None

        # Goal pose (updated from RViz "2D Goal Pose" button)
        self.goal_x: float | None = None
        self.goal_y: float | None = None

        # TF2 for frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to the map
        self.sub_map = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 1
        )

        # Subscribe to odometry for current robot position
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # Subscribe to RViz goal pose ("2D Goal Pose" button publishes to /goal_pose)
        self.sub_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 1
        )

        # Service to trigger planning
        self.srv_plan = self.create_service(
            Trigger, "plan_path", self.plan_path_callback
        )

        # Publisher for the planned path
        self.pub_path = self.create_publisher(Path, "/planned_path", 1)

        self.get_logger().info(
            "Global Planner ready.\n"
            "  - Set goal using RViz '2D Goal Pose' button\n"
            "  - Call 'plan_path' service to compute a path\n"
            "  - Robot start position is read from /odom"
        )

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg
        self.map_info = msg.info
        # Use int16 to avoid int8 overflow (-128 to 127 range can't represent 100 correctly)
        self.map_data = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )

        # Precompute a binary mask of free cells for planning
        # Only cells with value == 0 are considered free
        self.free_mask = self.map_data == 0

        self.get_logger().info(
            f"Map received: {msg.info.width}x{msg.info.height}, "
            f"free cells: {np.count_nonzero(self.free_mask)}, "
            f"occupied: {np.count_nonzero(self.map_data >= 50)}, "
            f"unknown: {np.count_nonzero(self.map_data == -1)}",
            throttle_duration_sec=5.0,
        )

    def odom_callback(self, msg: Odometry):
        """Update current robot position from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def goal_callback(self, msg: PoseStamped):
        """Update goal position from RViz '2D Goal Pose'.
        Transforms the goal into the odom frame to match the map."""

        # Determine the target frame from the map
        target_frame = "odom"

        # If the goal is already in odom, no transform needed
        if msg.header.frame_id == target_frame or msg.header.frame_id == "":
            self.goal_x = msg.pose.position.x
            self.goal_y = msg.pose.position.y
        else:
            try:
                # Transform goal pose into odom frame
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0),
                )
                transformed = tf2_geometry_msgs.do_transform_pose_stamped(
                    msg, transform
                )
                self.goal_x = transformed.pose.position.x
                self.goal_y = transformed.pose.position.y
            except Exception as e:
                self.get_logger().error(
                    f"Failed to transform goal from '{msg.header.frame_id}' to "
                    f"'{target_frame}': {e}"
                )
                return

        self.get_logger().info(
            f"Goal received (in {target_frame} frame): "
            f"({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

    def plan_path_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self.latest_map is None or self.map_data is None:
            response.success = False
            response.message = "No map received yet."
            self.get_logger().warn("Plan requested but no map has been received.")
            return response

        if self.robot_x is None or self.robot_y is None:
            response.success = False
            response.message = "No odometry received yet — robot position unknown."
            self.get_logger().warn(response.message)
            return response

        if self.goal_x is None or self.goal_y is None:
            response.success = False
            response.message = (
                "No goal set. Use RViz '2D Goal Pose' button to set a goal."
            )
            self.get_logger().warn(response.message)
            return response

        # Use current robot position as start
        start_x = self.robot_x
        start_y = self.robot_y
        goal_x = self.goal_x
        goal_y = self.goal_y

        self.get_logger().info(
            f"Planning from ({start_x:.2f}, {start_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})"
        )

        # Convert world coordinates to grid indices
        start_cell = self.world_to_grid(start_x, start_y)
        goal_cell = self.world_to_grid(goal_x, goal_y)

        # Validate that cells are within bounds
        if not self.is_valid_cell(start_cell):
            response.success = False
            response.message = (
                f"Start position ({start_x:.2f}, {start_y:.2f}) is outside the map."
            )
            self.get_logger().error(response.message)
            return response

        if not self.is_valid_cell(goal_cell):
            response.success = False
            response.message = (
                f"Goal position ({goal_x:.2f}, {goal_y:.2f}) is outside the map."
            )
            self.get_logger().error(response.message)
            return response

        # Validate that start and goal are on free cells
        sc, sr = start_cell
        gc, gr = goal_cell

        if not self.free_mask[sr, sc]:
            response.success = False
            response.message = (
                f"Start cell ({sc}, {sr}) is not free (value={self.map_data[sr, sc]}). "
                f"Robot may be on an occupied or unknown cell."
            )
            self.get_logger().error(response.message)
            return response

        if not self.free_mask[gr, gc]:
            response.success = False
            response.message = (
                f"Goal cell ({gc}, {gr}) is not free (value={self.map_data[gr, gc]}). "
                f"Choose a goal on explored free space."
            )
            self.get_logger().error(response.message)
            return response

        path_cells = self.compute_path(start_cell, goal_cell)

        if path_cells is None or len(path_cells) == 0:
            response.success = False
            response.message = "No path found."
            self.get_logger().warn(response.message)
            return response

        # Convert grid path to ROS Path message and publish
        path_msg = self.cells_to_path(path_cells)
        self.pub_path.publish(path_msg)

        # Save path overlayed on map as PNG
        output_dir = os.path.expanduser("~/rob599_ws/maps")
        os.makedirs(output_dir, exist_ok=True)
        self.save_path_image(path_cells, os.path.join(output_dir, "planned_path.png"))

        response.success = True
        response.message = f"Path found with {len(path_cells)} waypoints."
        self.get_logger().info(response.message)
        return response

    def compute_path(
        self, start: tuple[int, int], goal: tuple[int, int]
    ) -> list[tuple[int, int]] | None:
        """Compute a path from start to goal on the grid using A*.

        Args:
            start: (col, row) in grid coordinates.
            goal: (col, row) in grid coordinates.

        Returns:
            A list of (col, row) tuples representing the path, or None if no path exists.
        """
        goal_cell = Cell(goal[0], goal[1])

        start_cell = Cell(start[0], start[1], g=0.0)
        start_cell.h = start_cell.distance_to(goal_cell)
        start_cell.f = start_cell.g + start_cell.h

        # Open list as a min-heap (priority queue) ordered by f
        open_list: list[Cell] = []
        heapq.heappush(open_list, start_cell)

        # Closed set for O(1) lookup
        closed_set: set[Cell] = set()

        # Track best g-score per cell for duplicate detection
        g_scores: dict[tuple[int, int], float] = {(start_cell.col, start_cell.row): 0.0}

        while open_list:
            current = heapq.heappop(open_list)

            # Skip if already visited with a better cost
            if current in closed_set:
                continue

            # Goal reached — reconstruct path
            if current == goal_cell:
                path = []
                node = current
                while node is not None:
                    path.append((node.col, node.row))
                    node = node.parent
                path.reverse()
                return path

            closed_set.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue

                key = (neighbor.col, neighbor.row)
                temp_g = neighbor.g

                # Only process if this is a better path to the neighbor
                if temp_g < g_scores.get(key, float("inf")):
                    g_scores[key] = temp_g
                    neighbor.h = neighbor.distance_to(goal_cell)
                    neighbor.f = temp_g + neighbor.h
                    neighbor.parent = current
                    heapq.heappush(open_list, neighbor)

        self.get_logger().warn("A* exhausted all reachable cells — no path exists.")
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

    def get_neighbors(self, cell: Cell) -> list[Cell]:
        """Return valid neighboring cells (8-connected) that are free.

        Only cells explicitly marked as free (value == 0) are traversable.
        Occupied and unknown cells are pruned from the planning grid.

        Args:
            cell: The current cell.

        Returns:
            A list of neighboring Cell objects that are within bounds and free.
        """
        neighbors = []

        # 8-connected: cardinal + diagonal
        directions = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ]

        for dcol, drow in directions:
            nc = cell.col + dcol
            nr = cell.row + drow

            # Check bounds
            if not self.is_valid_cell((nc, nr)):
                continue

            # Only allow free cells (skip occupied AND unknown)
            if not self.free_mask[nr, nc]:
                continue

            # Diagonal movement costs sqrt(2), cardinal costs 1
            cost = math.sqrt(2) if (dcol != 0 and drow != 0) else 1.0

            neighbors.append(Cell(nc, nr, g=cell.g + cost))

        return neighbors

    def heuristic(self, a: Cell, b: Cell) -> float:
        """Calculate the Euclidean distance between two grid cells.

        Args:
            a: The first cell.
            b: The second cell.

        Returns:
            Euclidean distance between the two cells.
        """
        return a.distance_to(b)

    def save_path_image(
        self, path_cells: list[tuple[int, int]], output_path: str = "planned_path.png"
    ) -> None:
        """Draw the planned path on top of the map and save as a PNG.

        Args:
            path_cells: List of (col, row) tuples from compute_path.
            output_path: File path for the output PNG.
        """
        if self.map_data is None or self.map_info is None:
            self.get_logger().error("Cannot save path image — no map data.")
            return

        height, width = self.map_data.shape

        # --- Build color map image ---
        # Default unknown = light blue
        img_array = np.full((height, width, 3), [173, 216, 230], dtype=np.uint8)

        # Free -> white
        img_array[self.map_data == 0] = [254, 254, 254]

        # Occupied -> black
        img_array[self.map_data == 100] = [0, 0, 0]

        # Unknown -> light blue (already default)

        # Partially occupied -> grayscale
        mask = (self.map_data > 0) & (self.map_data < 100)
        gray = (((100 - self.map_data[mask].astype(np.int16)) * 254) // 100).astype(
            np.uint8
        )
        img_array[mask] = np.stack([gray, gray, gray], axis=-1)

        # Flip vertically (map origin bottom-left, image origin top-left)
        img_array = np.flipud(img_array)

        # --- Draw path on top ---
        img = Image.fromarray(img_array, mode="RGB")
        draw = ImageDraw.Draw(img)

        if len(path_cells) >= 2:
            # Convert (col, row) to image pixel coordinates (col, flipped_row)
            pixel_coords = [(col, height - 1 - row) for col, row in path_cells]

            # Draw path line in red
            draw.line(pixel_coords, fill=(255, 0, 0), width=2)

        # Draw start point in green
        if path_cells:
            sc, sr = path_cells[0]
            sr_img = height - 1 - sr
            r = 3  # marker radius
            draw.ellipse([sc - r, sr_img - r, sc + r, sr_img + r], fill=(0, 200, 0))

        # Draw goal point in blue
        if len(path_cells) > 1:
            gc, gr = path_cells[-1]
            gr_img = height - 1 - gr
            draw.ellipse([gc - r, gr_img - r, gc + r, gr_img + r], fill=(0, 0, 255))

        img.save(output_path)
        self.get_logger().info(f"Path image saved to {output_path}")


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
