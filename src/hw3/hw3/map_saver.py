#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
import numpy as np
from PIL import Image
import yaml
import os
from datetime import datetime


class MapSaver(Node):
    def __init__(self):
        super().__init__("map_saver")

        self.declare_parameter(
            "output_dir", os.path.expanduser("~/rob599_ws/src/hw3/maps")
        )
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("file_prefix", "map")

        self.output_dir = self.get_parameter("output_dir").value
        map_topic = self.get_parameter("map_topic").value
        self.file_prefix = self.get_parameter("file_prefix").value

        # Store latest map
        self.latest_map: OccupancyGrid | None = None

        self.sub_map = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, 1
        )

        self.srv_save = self.create_service(Trigger, "save_map", self.save_map_callback)

        self.get_logger().info(
            f"Map Saver ready. Call 'save_map' service to save the latest map from '{map_topic}'."
        )

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def save_map_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self.latest_map is None:
            response.success = False
            response.message = "No map received yet."
            self.get_logger().warn("Save requested but no map has been received.")
            return response

        try:
            os.makedirs(self.output_dir, exist_ok=True)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_name = f"{self.file_prefix}_{timestamp}"
            pgm_path = os.path.join(self.output_dir, f"{base_name}.pgm")
            yaml_path = os.path.join(self.output_dir, f"{base_name}.yaml")

            grid = self.latest_map
            width = grid.info.width
            height = grid.info.height
            resolution = grid.info.resolution
            origin = grid.info.origin

            # Convert OccupancyGrid data to image
            # OccupancyGrid values: -1 = unknown, 0 = free, 100 = occupied
            # PGM convention (ROS map_server): 254 = free (white), 0 = occupied (black), 205 = unknown (grey)
            data = np.array(grid.data, dtype=np.int8).reshape((height, width))
            img = np.full((height, width), 205, dtype=np.uint8)  # default unknown

            img[data == 0] = 254  # free -> white
            img[data == 100] = 0  # occupied -> black
            img[data == -1] = 205  # unknown -> grey

            # For values between 0 and 100, scale linearly
            mask = (data > 0) & (data < 100)
            img[mask] = (((100 - data[mask].astype(np.int16)) * 254) // 100).astype(
                np.uint8
            )

            # Flip vertically (ROS map origin is bottom-left, image origin is top-left)
            img = np.flipud(img)

            # Save PGM
            pil_img = Image.fromarray(img, mode="L")
            pil_img.save(pgm_path)

            # --- PNG (color with light blue unknown) ---
            white = [254, 254, 254]
            black = [0, 0, 0]
            light_blue = [173, 216, 230]
            img_color = np.full(
                (height, width, 3), light_blue, dtype=np.uint8
            )  # default unknown = light blue

            img_color[data == 0] = white  # free -> white
            img_color[data == 100] = black  # occupied -> black
            img_color[data == -1] = light_blue  # unknown -> light blue

            # For values between 0 and 100, scale linearly (grayscale)
            gray = (((100 - data[mask].astype(np.int16)) * 254) // 100).astype(np.uint8)
            img_color[mask] = np.stack([gray, gray, gray], axis=-1)

            # Flip vertically
            img_color = np.flipud(img_color)

            # Save PNG
            png_path = os.path.join(self.output_dir, f"{base_name}.png")
            pil_img_color = Image.fromarray(img_color, mode="RGB")
            pil_img_color.save(png_path)

            # Save YAML metadata
            yaml_data = {
                "image": f"{base_name}.pgm",
                "resolution": float(resolution),
                "origin": [
                    float(origin.position.x),
                    float(origin.position.y),
                    0.0,
                ],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }

            with open(yaml_path, "w") as f:
                yaml.dump(yaml_data, f, default_flow_style=False)

            response.success = True
            response.message = f"Map saved to {yaml_path}, {pgm_path}, and {png_path}"
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Failed to save map: {e}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
