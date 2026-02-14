#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import numpy as np
import math
import time

from hw3.utils import get_yaw_from_quaternion


class BayesianMapper(Node):
    def __init__(self):
        super().__init__("bayesian_mapper")

        self.declare_parameter("width", 50.0)  # meters
        self.declare_parameter("height", 50.0)  # meters
        self.declare_parameter("resolution", 0.05)  # meters/cell
        self.declare_parameter("laser_scan_topic", "/base_scan")
        self.declare_parameter("publish_interval", 1.0)  # seconds between map publishes
        self.declare_parameter("ray_stride", 3)  # process every Nth ray

        self.width_m = self.get_parameter("width").value
        self.height_m = self.get_parameter("height").value
        self.resolution = self.get_parameter("resolution").value
        laser_scan_topic = self.get_parameter("laser_scan_topic").value
        self.publish_interval = self.get_parameter("publish_interval").value
        self.ray_stride = self.get_parameter("ray_stride").value

        self.cols = int(self.width_m / self.resolution)
        self.rows = int(self.height_m / self.resolution)

        # Origin is the position of the cell (0,0) in the map
        # We'll center the map on (0,0) of the world
        self.origin_x = -self.width_m / 2.0
        self.origin_y = -self.height_m / 2.0

        # Log odds map
        self.l_map = np.zeros((self.rows, self.cols), dtype=np.float32)

        # Visited mask for extra credit (unknown area)
        self.visited = np.zeros((self.rows, self.cols), dtype=bool)

        # Parameters for update
        self.l_occ = np.float32(0.85)
        self.l_free = np.float32(-0.85)

        # Max/Min log odds to prevent saturation issues
        self.l_max = np.float32(5.0)
        self.l_min = np.float32(-5.0)

        # Occupancy protection: after n occupied updates, freeze the cell
        self.occ_hit_count = np.zeros((self.rows, self.cols), dtype=np.int16)
        self.occ_protect_thresh = 10

        self.current_pose = None
        self.last_publish_time = 0.0

        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.sub_scan = self.create_subscription(
            LaserScan, laser_scan_topic, self.scan_callback, 10
        )

        self.pub_map = self.create_publisher(OccupancyGrid, "/map", 1)

        # Pre-compute inverse resolution
        self.inv_resolution = 1.0 / self.resolution

        self.get_logger().info("Bayesian Mapper Initialized")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        if self.current_pose is None:
            return

        # Get robot pose
        px = self.current_pose.position.x
        py = self.current_pose.position.y
        robot_yaw = get_yaw_from_quaternion(self.current_pose.orientation)

        # Robot position in grid coords
        r_c = int((px - self.origin_x) * self.inv_resolution)
        r_r = int((py - self.origin_y) * self.inv_resolution)

        # Convert ranges to numpy array and subsample
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = (
            np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
            + msg.angle_min
        )

        # Subsample rays
        indices = np.arange(0, len(ranges), self.ray_stride)
        ranges = ranges[indices]
        angles = angles[indices]

        # Filter invalid ranges
        valid = np.isfinite(ranges) & (ranges >= msg.range_min)
        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) == 0:
            return

        # Separate max-range vs hit rays
        is_max = ranges >= msg.range_max
        ray_angles = robot_yaw + angles

        # Compute endpoints in world coords
        hx = px + ranges * np.cos(ray_angles)
        hy = py + ranges * np.sin(ray_angles)

        # Convert to grid coords
        h_c = ((hx - self.origin_x) * self.inv_resolution).astype(int)
        h_r = ((hy - self.origin_y) * self.inv_resolution).astype(int)

        # Process each ray using fast bresenham
        frozen = self.occ_hit_count >= self.occ_protect_thresh

        for i in range(len(ranges)):
            cells = self._bresenham_fast(r_c, r_r, h_c[i], h_r[i])
            if len(cells) == 0:
                continue

            cols_arr = cells[:, 0]
            rows_arr = cells[:, 1]

            # Bounds check
            mask = (
                (cols_arr >= 0)
                & (cols_arr < self.cols)
                & (rows_arr >= 0)
                & (rows_arr < self.rows)
            )

            if is_max[i]:
                # All cells are free
                vc = cols_arr[mask]
                vr = rows_arr[mask]
                not_frozen = ~frozen[vr, vc]
                vc = vc[not_frozen]
                vr = vr[not_frozen]
                self.l_map[vr, vc] += self.l_free
                self.visited[vr, vc] = True
            else:
                # Free cells (all but last)
                if len(cols_arr) > 1:
                    free_mask = mask[:-1]
                    fc = cols_arr[:-1][free_mask]
                    fr = rows_arr[:-1][free_mask]
                    not_frozen_free = ~frozen[fr, fc]
                    fc = fc[not_frozen_free]
                    fr = fr[not_frozen_free]
                    self.l_map[fr, fc] += self.l_free
                    # Decrement occ counter for free updates
                    dec_mask = self.occ_hit_count[fr, fc] > 0
                    self.occ_hit_count[fr[dec_mask], fc[dec_mask]] -= 1
                    self.visited[fr, fc] = True

                # Occupied cell (last)
                lc, lr = cols_arr[-1], rows_arr[-1]
                if mask[-1]:
                    if not frozen[lr, lc]:
                        self.l_map[lr, lc] += self.l_occ
                        self.occ_hit_count[lr, lc] += 1
                    self.visited[lr, lc] = True

        # Clamp values
        np.clip(self.l_map, self.l_min, self.l_max, out=self.l_map)

        # Throttled publish
        now = time.monotonic()
        if now - self.last_publish_time >= self.publish_interval:
            self.publish_map()
            self.last_publish_time = now

    @staticmethod
    def _bresenham_fast(x0, y0, x1, y1):
        """Bresenham's line algorithm returning an Nx2 numpy array."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        n = max(dx, dy) + 1
        points = np.empty((n, 2), dtype=np.int32)
        idx = 0
        x, y = x0, y0

        if dx > dy:
            err = dx >> 1
            while x != x1:
                points[idx, 0] = x
                points[idx, 1] = y
                idx += 1
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy >> 1
            while y != y1:
                points[idx, 0] = x
                points[idx, 1] = y
                idx += 1
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        points[idx, 0] = x1
        points[idx, 1] = y1
        idx += 1

        return points[:idx]

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
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

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
