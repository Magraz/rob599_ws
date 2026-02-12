#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import numpy as np
import math

from hw3.utils import get_yaw_from_quaternion


class BayesianMapper(Node):
    def __init__(self):
        super().__init__("bayesian_mapper")

        self.declare_parameter("width", 50.0)  # meters
        self.declare_parameter("height", 50.0)  # meters
        self.declare_parameter("resolution", 0.05)  # meters/cell

        self.width_m = self.get_parameter("width").value
        self.height_m = self.get_parameter("height").value
        self.resolution = self.get_parameter("resolution").value

        self.cols = int(self.width_m / self.resolution)
        self.rows = int(self.height_m / self.resolution)

        # Origin is the position of the cell (0,0) in the map
        # We'll center the map on (0,0) of the world
        self.origin_x = -self.width_m / 2.0
        self.origin_y = -self.height_m / 2.0

        # Log odds map
        # Prior probability 0.5 -> log odds 0.0
        self.l_map = np.zeros((self.rows, self.cols))

        # Visited mask for extra credit (unknown area)
        self.visited = np.zeros((self.rows, self.cols), dtype=bool)

        # Parameters for update
        # p_occ = 0.7 => log(0.7/0.3) ~= 0.847
        # p_free = 0.3 => log(0.3/0.7) ~= -0.847
        self.l_occ = 0.85
        self.l_free = -0.85

        # Max/Min log odds to prevent saturation issues (optional)
        self.l_max = 5.0
        self.l_min = -5.0

        self.current_pose = None

        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.sub_scan = self.create_subscription(
            LaserScan, "/base_scan", self.scan_callback, 10
        )

        self.pub_map = self.create_publisher(OccupancyGrid, "/map", 1)

        self.get_logger().info("Bayesian Mapper Initialized")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        if self.current_pose is None:
            return

        # Get robot pose
        px = self.current_pose.position.x
        py = self.current_pose.position.y
        orientation = self.current_pose.orientation

        # Convert quaternion to yaw
        robot_yaw = get_yaw_from_quaternion(orientation)

        # Robot position in grid coords
        r_c, r_r = self.world_to_grid(px, py)

        # Process each ray
        angle = msg.angle_min

        updates = {}  # Store updates in a dict to handle overlaps?
        # Or just apply sequentially. Sequential is fine but order matters if multiple rays hit same cell.
        # But usually we process ranges independently.
        # Actually, standard occupancy grid mapping often computes update per ray.

        # To avoid re-calculating cells multiple times for one scan (if rays overlap heavily near robot),
        # we can collect them. But simple implementation is per ray.

        for r in msg.ranges:

            # Skip invalid ranges
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            # Check range limits
            if r > msg.range_max:
                # Treat as free up to max? or just skip?
                # Usually max range means free up to max.
                # But let's stick to valid hits for simplicity, or clamp.
                # If r is inf/max, it means empty space.
                pass

            # If r < range_min, ignore
            if r < msg.range_min:
                angle += msg.angle_increment
                continue

            # Calculate hit point
            # Current ray angle in world frame
            # Assuming laser is at robot base (no offset)
            ray_angle = robot_yaw + angle

            hx = px + r * math.cos(ray_angle)
            hy = py + r * math.sin(ray_angle)

            h_c, h_r = self.world_to_grid(hx, hy)

            # Raytrace
            cells = self.bresenham(r_c, r_r, h_c, h_r)

            for c, r_idx in cells[:-1]:
                if self.is_valid_index(c, r_idx):
                    self.l_map[r_idx, c] += self.l_free
                    self.visited[r_idx, c] = True

            # Last cell is occupied
            last_c, last_r = cells[-1]
            if self.is_valid_index(last_c, last_r):
                self.l_map[last_r, last_c] += self.l_occ
                self.visited[last_r, last_c] = True

            angle += msg.angle_increment

        # Clamp values
        np.clip(self.l_map, self.l_min, self.l_max, out=self.l_map)

        # Publish map
        self.publish_map()

    def world_to_grid(self, x, y):
        c = int((x - self.origin_x) / self.resolution)
        r = int((y - self.origin_y) / self.resolution)
        return c, r

    def is_valid_index(self, c, r):
        return 0 <= c < self.cols and 0 <= r < self.rows

    def bresenham(self, x0, y0, x1, y1):
        """Bresenham's Line Algorithm
        Produces a list of tuples from start and end
        """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
            points.append((x, y))
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
            points.append((x, y))

        points.append((x1, y1))

        return points

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = (
            "odom"  # Assuming our grid is attached to 'map' and odom is map-referenced
        )

        msg.info.resolution = self.resolution
        msg.info.width = self.cols
        msg.info.height = self.rows
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Convert log odds to probability
        # Use numpy for speed
        exp_l = np.exp(self.l_map)
        probs = 1.0 - 1.0 / (1.0 + exp_l)

        # Scale to 0-100
        grid_data = (probs * 100).astype(np.int8)

        # Apply visited mask (Extra Credit)
        # Note: -1 is int8 255 or -1? In python bytes/int8 list, usually -1 is fine.
        # ROS message data is int8[].

        # Where not visited, set to -1
        grid_data[~self.visited] = -1

        # Flatten
        msg.data = grid_data.flatten().tolist()

        self.pub_map.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BayesianMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
