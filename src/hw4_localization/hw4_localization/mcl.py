#!/usr/bin/env python3
"""
Monte Carlo Localization (MCL) node for ROS 2.

Implements a particle filter for robot localization using:
- Differential drive motion model (odometry-based)
- Likelihood field sensor model (laser scan + occupancy grid)
- Low-variance resampling

Subscribes:
    /map              (nav_msgs/OccupancyGrid)  - static map from map_server
    /base_scan        (sensor_msgs/LaserScan)    - laser range data
    /odom             (nav_msgs/Odometry)         - wheel odometry
    /initialpose      (geometry_msgs/PoseWithCovarianceStamped) - manual pose reset

Publishes:
    ~/particle_cloud  (geometry_msgs/PoseArray)   - current particle set
    ~/pose            (geometry_msgs/PoseStamped)  - estimated robot pose
"""

import math

import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    Pose,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

import tf2_ros
from geometry_msgs.msg import TransformStamped


def yaw_from_quaternion(q):
    """Extract yaw from a quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    """Return (x, y, z, w) quaternion for a given yaw angle."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MCL(Node):
    def __init__(self):
        super().__init__("mcl")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("num_particles", 1000)
        self.declare_parameter("alpha1", 0.2)  # rot  -> rot  noise
        self.declare_parameter("alpha2", 0.2)  # trans -> rot  noise
        self.declare_parameter("alpha3", 0.2)  # trans -> trans noise
        self.declare_parameter("alpha4", 0.2)  # rot  -> trans noise
        self.declare_parameter("sigma_hit", 0.2)
        self.declare_parameter("z_hit", 0.95)
        self.declare_parameter("z_rand", 0.05)
        self.declare_parameter("laser_max_range", 10.0)
        self.declare_parameter("max_beams", 60)
        self.declare_parameter("update_min_d", 0.2)  # metres before update
        self.declare_parameter("update_min_a", 0.2)  # radians before update
        self.declare_parameter("resample_interval", 1)  # updates between resamples
        self.declare_parameter("publish_rate", 10.0)  # Hz for particle cloud
        self.declare_parameter("scan_topic", "/base_scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("tf_broadcast", True)
        self.declare_parameter("global_frame_id", "map")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")

        self.num_particles = self.get_parameter("num_particles").value
        self.alpha1 = self.get_parameter("alpha1").value
        self.alpha2 = self.get_parameter("alpha2").value
        self.alpha3 = self.get_parameter("alpha3").value
        self.alpha4 = self.get_parameter("alpha4").value
        self.sigma_hit = self.get_parameter("sigma_hit").value
        self.z_hit = self.get_parameter("z_hit").value
        self.z_rand = self.get_parameter("z_rand").value
        self.laser_max_range = self.get_parameter("laser_max_range").value
        self.max_beams = self.get_parameter("max_beams").value
        self.update_min_d = self.get_parameter("update_min_d").value
        self.update_min_a = self.get_parameter("update_min_a").value
        self.resample_interval = self.get_parameter("resample_interval").value
        self.publish_rate = self.get_parameter("publish_rate").value
        scan_topic = self.get_parameter("scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.do_tf_broadcast = self.get_parameter("tf_broadcast").value
        self.global_frame_id = self.get_parameter("global_frame_id").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value

        # ── State ───────────────────────────────────────────────────
        # Particles: Nx3 array  [x, y, theta]
        self.particles = None
        self.weights = None
        self.map_data = None  # OccupancyGrid message
        self.map_grid = None  # 2-D numpy bool (True = occupied)
        self.likelihood_field = None  # distance transform of map
        self.map_info = None  # MapMetaData

        self.prev_odom = None  # last Odometry used for motion update
        self.latest_odom = None  # most recent odom msg (stored before init too)
        self.updates_since_resample = 0
        self.motion_pending = (
            False  # True after motion update, cleared after sensor update
        )
        self.initialised = False

        # ── TF broadcaster ──────────────────────────────────────────
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Publishers ──────────────────────────────────────────────
        self.particle_pub = self.create_publisher(PoseArray, "~/particle_cloud", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "~/pose", 10)

        # ── Subscribers ─────────────────────────────────────────────
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)
        self.create_subscription(
            LaserScan, scan_topic, self._scan_cb, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._initialpose_cb, 10
        )

        # ── Publish timer ───────────────────────────────────────────
        self.create_timer(1.0 / self.publish_rate, self._publish_particles)

        self.get_logger().info(
            f"MCL started: {self.num_particles} particles, "
            f"scan={scan_topic}, odom={odom_topic}"
        )

    # ================================================================
    #  Callbacks
    # ================================================================

    def _map_cb(self, msg: OccupancyGrid):
        """Process incoming occupancy grid and build likelihood field."""
        self.map_data = msg
        self.map_info = msg.info
        w, h = msg.info.width, msg.info.height

        # Convert to 2-D grid: True where occupied (value >= 50)
        raw = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self.map_grid = raw >= 50

        # Likelihood field: Euclidean distance to nearest occupied cell (in pixels)
        # Then scale to metres.
        dist_pixels = distance_transform_edt(~self.map_grid)
        self.likelihood_field = dist_pixels * self.map_info.resolution

        self.get_logger().info(
            f"Map received: {w}x{h}, resolution={self.map_info.resolution:.3f} m/px"
        )

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped):
        """Re-initialise particles around a given pose."""
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        # Spread from covariance diagonal
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        std_x = math.sqrt(max(cov[0, 0], 1e-6))
        std_y = math.sqrt(max(cov[1, 1], 1e-6))
        std_yaw = math.sqrt(max(cov[5, 5], 1e-6))

        self.particles = np.column_stack(
            [
                np.random.normal(px, std_x, self.num_particles),
                np.random.normal(py, std_y, self.num_particles),
                np.random.normal(yaw, std_yaw, self.num_particles),
            ]
        )
        self.weights = np.ones(self.num_particles) / self.num_particles
        self.prev_odom = None
        self.initialised = True
        self.get_logger().info(
            f"Particles initialised around ({px:.2f}, {py:.2f}, {math.degrees(yaw):.1f} deg)"
        )

        # Publish an immediate map→odom TF so the frames are connected
        # before the first motion+scan cycle.
        self._publish_estimated_pose(self.get_clock().now().to_msg())

    def _odom_cb(self, msg: Odometry):
        """Store latest odometry for the motion model."""
        self.latest_odom = msg  # always track, even before initialisation

        if not self.initialised:
            return

        if self.prev_odom is None:
            self.prev_odom = msg
            return

        # Compute incremental motion in the odom frame
        dx, dy, prev_theta, dtheta = self._compute_odom_delta(self.prev_odom, msg)

        # Only run the full update if the robot moved enough
        dist = math.hypot(dx, dy)
        if dist < self.update_min_d and abs(dtheta) < self.update_min_a:
            return

        # ── Motion update ───────────────────────────────────────
        self._motion_update(dx, dy, prev_theta, dtheta)
        self.prev_odom = msg
        self.motion_pending = True

    def _scan_cb(self, msg: LaserScan):
        """Run sensor update + resampling only after the robot has moved."""
        if not self.initialised or self.likelihood_field is None:
            return

        # Only update weights when a motion update has occurred
        if not self.motion_pending:
            return
        self.motion_pending = False

        # ── Sensor update ───────────────────────────────────────
        self._sensor_update(msg)

        # ── Resampling ──────────────────────────────────────────
        self.updates_since_resample += 1
        if self.updates_since_resample >= self.resample_interval:
            self._low_variance_resample()
            self.updates_since_resample = 0

        # ── Publish estimated pose ──────────────────────────────
        self._publish_estimated_pose(msg.header.stamp)

    # ================================================================
    #  Motion model  (odometry-based, differential drive)
    # ================================================================

    @staticmethod
    def _compute_odom_delta(prev: Odometry, curr: Odometry):
        """Return (dx, dy, prev_theta, dtheta) between two odom msgs."""
        x0 = prev.pose.pose.position.x
        y0 = prev.pose.pose.position.y
        th0 = yaw_from_quaternion(prev.pose.pose.orientation)

        x1 = curr.pose.pose.position.x
        y1 = curr.pose.pose.position.y
        th1 = yaw_from_quaternion(curr.pose.pose.orientation)

        dx = x1 - x0
        dy = y1 - y0
        dtheta = math.atan2(math.sin(th1 - th0), math.cos(th1 - th0))
        return dx, dy, th0, dtheta

    def _motion_update(self, dx, dy, prev_theta, dtheta):
        """Apply the odometry motion model to all particles (vectorised)."""
        trans = math.hypot(dx, dy)
        if trans < 1e-6 and abs(dtheta) < 1e-6:
            return

        # Decompose into rot1 -> translation -> rot2
        # rot1 is relative to the robot's PREVIOUS heading (Probabilistic Robotics Table 5.6)
        rot1 = math.atan2(dy, dx) - prev_theta if trans > 1e-6 else 0.0
        rot1 = math.atan2(math.sin(rot1), math.cos(rot1))  # normalise
        rot2 = dtheta - rot1
        rot2 = math.atan2(math.sin(rot2), math.cos(rot2))  # normalise

        # Noise standard deviations (Probabilistic Robotics Table 5.6)
        sd_rot1 = math.sqrt(self.alpha1 * rot1**2 + self.alpha2 * trans**2)
        sd_trans = math.sqrt(self.alpha3 * trans**2 + self.alpha4 * (rot1**2 + rot2**2))
        sd_rot2 = math.sqrt(self.alpha1 * rot2**2 + self.alpha2 * trans**2)

        n = self.num_particles
        noisy_rot1 = rot1 - np.random.normal(0, max(sd_rot1, 1e-6), n)
        noisy_trans = trans - np.random.normal(0, max(sd_trans, 1e-6), n)
        noisy_rot2 = rot2 - np.random.normal(0, max(sd_rot2, 1e-6), n)

        self.particles[:, 0] += noisy_trans * np.cos(self.particles[:, 2] + noisy_rot1)
        self.particles[:, 1] += noisy_trans * np.sin(self.particles[:, 2] + noisy_rot1)
        self.particles[:, 2] += noisy_rot1 + noisy_rot2
        # Normalise angles
        self.particles[:, 2] = np.arctan2(
            np.sin(self.particles[:, 2]), np.cos(self.particles[:, 2])
        )

    # ================================================================
    #  Sensor model  (likelihood field)
    # ================================================================

    def _sensor_update(self, scan: LaserScan):
        """Compute particle weights using the likelihood field model."""
        info = self.map_info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        h, w = self.likelihood_field.shape

        # Sub-sample scan beams
        num_ranges = len(scan.ranges)
        if num_ranges == 0:
            return
        step = max(1, num_ranges // self.max_beams)
        beam_indices = range(0, num_ranges, step)

        log_weights = np.zeros(self.num_particles)

        for idx in beam_indices:
            r = scan.ranges[idx]
            if (
                math.isnan(r)
                or math.isinf(r)
                or r <= scan.range_min
                or r >= self.laser_max_range
            ):
                continue

            angle = scan.angle_min + idx * scan.angle_increment

            # Endpoint of this beam for every particle
            ex = self.particles[:, 0] + r * np.cos(self.particles[:, 2] + angle)
            ey = self.particles[:, 1] + r * np.sin(self.particles[:, 2] + angle)

            # Convert to grid coordinates
            mx = ((ex - ox) / res).astype(int)
            my = ((ey - oy) / res).astype(int)

            # Clamp to map bounds
            mx = np.clip(mx, 0, w - 1)
            my = np.clip(my, 0, h - 1)

            # Look up distance in the likelihood field
            dist = self.likelihood_field[my, mx]

            # Gaussian probability
            pz = self.z_hit * np.exp(-0.5 * (dist / self.sigma_hit) ** 2)
            pz += self.z_rand / self.laser_max_range
            # Avoid log(0)
            pz = np.maximum(pz, 1e-300)
            log_weights += np.log(pz)

        # Convert log-weights to weights
        log_weights -= np.max(log_weights)  # numerical stability
        self.weights = np.exp(log_weights)
        total = self.weights.sum()
        if total > 0:
            self.weights /= total
        else:
            self.weights = np.ones(self.num_particles) / self.num_particles

    # ================================================================
    #  Resampling  (low-variance / systematic)
    # ================================================================

    def _low_variance_resample(self):
        """Low-variance resampling (Probabilistic Robotics Table 4.4)."""
        n = self.num_particles
        r = np.random.uniform(0, 1.0 / n)
        c = self.weights[0]
        i = 0
        new_particles = np.empty_like(self.particles)

        for m in range(n):
            u = r + m / n
            while u > c:
                i += 1
                if i >= n:
                    i = n - 1
                    break
                c += self.weights[i]
            new_particles[m] = self.particles[i]

        self.particles = new_particles
        self.weights = np.ones(n) / n

    # ================================================================
    #  Publishing
    # ================================================================

    def _publish_particles(self):
        """Publish PoseArray of current particle set."""
        if self.particles is None:
            return

        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame_id

        for i in range(self.num_particles):
            p = Pose()
            p.position.x = float(self.particles[i, 0])
            p.position.y = float(self.particles[i, 1])
            qx, qy, qz, qw = quaternion_from_yaw(float(self.particles[i, 2]))
            p.orientation.x = qx
            p.orientation.y = qy
            p.orientation.z = qz
            p.orientation.w = qw
            msg.poses.append(p)

        self.particle_pub.publish(msg)

    def _publish_estimated_pose(self, stamp):
        """Compute weighted mean pose and publish it."""
        # Weighted mean of x, y
        mean_x = float(np.average(self.particles[:, 0], weights=self.weights))
        mean_y = float(np.average(self.particles[:, 1], weights=self.weights))

        # Circular mean for theta
        mean_cos = float(np.average(np.cos(self.particles[:, 2]), weights=self.weights))
        mean_sin = float(np.average(np.sin(self.particles[:, 2]), weights=self.weights))
        mean_theta = math.atan2(mean_sin, mean_cos)

        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.global_frame_id
        pose_msg.pose.position.x = mean_x
        pose_msg.pose.position.y = mean_y
        qx, qy, qz, qw = quaternion_from_yaw(mean_theta)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        # Broadcast map -> odom TF
        odom_src = self.prev_odom or self.latest_odom
        if self.do_tf_broadcast and odom_src is not None:
            self._broadcast_tf(mean_x, mean_y, mean_theta, stamp, odom_src)

    def _broadcast_tf(self, map_x, map_y, map_theta, stamp, odom):
        """Broadcast the map -> odom transform based on the estimated pose and current odom."""
        odom_x = odom.pose.pose.position.x
        odom_y = odom.pose.pose.position.y
        odom_theta = yaw_from_quaternion(odom.pose.pose.orientation)

        # map_pose = map_to_odom * odom_pose
        # => map_to_odom = map_pose * inv(odom_pose)
        cos_o = math.cos(odom_theta)
        sin_o = math.sin(odom_theta)
        # Inverse of odom pose
        inv_x = -(cos_o * odom_x + sin_o * odom_y)
        inv_y = -(-sin_o * odom_x + cos_o * odom_y)
        inv_theta = -odom_theta

        # Compose: map_pose * inv(odom)
        cos_m = math.cos(map_theta)
        sin_m = math.sin(map_theta)
        tf_x = cos_m * inv_x - sin_m * inv_y + map_x
        tf_y = sin_m * inv_x + cos_m * inv_y + map_y
        tf_theta = map_theta + inv_theta

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.global_frame_id
        t.child_frame_id = self.odom_frame_id
        t.transform.translation.x = tf_x
        t.transform.translation.y = tf_y
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(tf_theta)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MCL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
