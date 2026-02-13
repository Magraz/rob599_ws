#!/usr/bin/env python3

import os

import numpy as np
import rclpy
import yaml
from PIL import Image
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf2_ros import Buffer, TransformListener


class MapLoader(Node):
    def __init__(self):
        super().__init__("map_loader")

        # Parameters
        self.declare_parameter("yaml_path", "")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("publish_rate", 1.0)

        self.yaml_path = str(self.get_parameter("yaml_path").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        if not self.yaml_path:
            self.get_logger().error("No yaml_path parameter provided. Shutting down.")
            raise SystemExit("yaml_path is required")

        # TF2 listener to get the map frame origin
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Use transient local durability so late subscribers get the map
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_map = self.create_publisher(OccupancyGrid, "/map", qos)

        self.occupancy_grid = None
        self.map_published = False

        # Wait for the TF to become available, then load and publish
        self.init_timer = self.create_timer(0.5, self._wait_for_tf)

        self.get_logger().info("Map Loader initialized. Waiting for /map TF frame...")

    def _wait_for_tf(self):
        """Poll until the map frame transform is available."""
        try:
            # Look up map frame origin in world (identity, but gives us the frame exists)
            t = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
        except Exception:
            self.get_logger().info(
                "Waiting for map -> odom transform...", throttle_duration_sec=5.0
            )
            return

        # Got the transform — extract the map frame origin
        map_origin_x = t.transform.translation.x
        map_origin_y = t.transform.translation.y
        map_origin_yaw = 2.0 * np.arctan2(
            t.transform.rotation.z, t.transform.rotation.w
        )

        self.get_logger().info(
            f"Got map frame origin: ({map_origin_x:.2f}, {map_origin_y:.2f}, yaw={map_origin_yaw:.3f})"
        )

        # Stop polling
        self.init_timer.cancel()

        # Load the map image and metadata
        self.occupancy_grid = self._load_map(
            self.yaml_path, map_origin_x, map_origin_y, map_origin_yaw
        )

        # Publish once immediately
        self._publish_map()

        # Optionally keep republishing
        if self.publish_rate > 0.0:
            period = 1.0 / self.publish_rate
            self.pub_timer = self.create_timer(period, self._publish_map)

        self.get_logger().info("Map loaded and published successfully.")

    def _load_map(
        self,
        yaml_path: str,
        map_origin_x: float,
        map_origin_y: float,
        map_origin_yaw: float,
    ) -> OccupancyGrid:
        """Load a map YAML + image file and return an OccupancyGrid message."""

        # --- Parse YAML ---
        with open(yaml_path, "r") as f:
            meta = yaml.safe_load(f)

        image_file = meta["image"]
        resolution = float(meta["resolution"])
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        negate = int(meta.get("negate", 0))
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        free_thresh = float(meta.get("free_thresh", 0.196))

        # Resolve image path relative to YAML file
        yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
        image_path = os.path.join(yaml_dir, image_file)

        if not os.path.isfile(image_path):
            self.get_logger().error(f"Image file not found: {image_path}")
            raise FileNotFoundError(f"Image file not found: {image_path}")

        # --- Load image ---
        img = Image.open(image_path).convert("L")
        img_array = np.array(img, dtype=np.float64)

        # Flip vertically (image origin top-left, map origin bottom-left)
        img_array = np.flipud(img_array)

        height, width = img_array.shape

        # --- Convert pixel values to occupancy ---
        if negate:
            img_array = 255.0 - img_array
        img_norm = img_array / 255.0

        occ_prob = 1.0 - img_norm

        grid_data = np.full(height * width, -1, dtype=np.int8)
        flat_occ = occ_prob.flatten()

        grid_data[flat_occ >= occupied_thresh] = 100
        grid_data[flat_occ <= free_thresh] = 0

        # No TF needed — just use the YAML origin directly
        grid_origin_x = float(origin[0])
        grid_origin_y = float(origin[1])
        grid_origin_yaw = float(origin[2]) if len(origin) >= 3 else 0.0
        if len(origin) >= 3:
            grid_origin_yaw += float(origin[2])

        # --- Build OccupancyGrid message ---
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = grid_origin_x
        msg.info.origin.position.y = grid_origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.z = np.sin(grid_origin_yaw / 2.0)
        msg.info.origin.orientation.w = np.cos(grid_origin_yaw / 2.0)

        msg.data = grid_data.tolist()

        self.get_logger().info(
            f"Map loaded: {width}x{height} @ {resolution} m/cell, "
            f"grid origin=({grid_origin_x:.2f}, {grid_origin_y:.2f}, yaw={grid_origin_yaw:.3f})"
        )

        return msg

    def _publish_map(self):
        if self.occupancy_grid is not None:
            self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
            self.pub_map.publish(self.occupancy_grid)


def main(args=None):
    rclpy.init(args=args)
    node = MapLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
