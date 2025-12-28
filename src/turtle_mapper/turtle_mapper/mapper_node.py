import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

import tf2_ros


def yaw_from_quat(qx, qy, qz, qw):
    # yaw (rotation around z) from quaternion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def bresenham(x0, y0, x1, y1):
    """Grid traversal from (x0,y0) to (x1,y1) inclusive."""
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x1 >= x0 else -1
    sy = 1 if y1 >= y0 else -1

    if dx >= dy:
        err = dx / 2.0
        while x != x1:
            cells.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            cells.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    cells.append((x1, y1))
    return cells


class OccupancyGridMapper(Node):
    def __init__(self):
        super().__init__("mapper_node")

        # ----- Map params (tune if needed) -----
        self.resolution = 0.05          # m/cell
        self.size_x_m = 12.0            # map width in meters
        self.size_y_m = 12.0            # map height in meters
        self.width = int(self.size_x_m / self.resolution)
        self.height = int(self.size_y_m / self.resolution)

        # Map origin (bottom-left) in odom frame
        self.origin_x = -self.size_x_m / 2.0
        self.origin_y = -self.size_y_m / 2.0

        # Log-odds values
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.l_occ = 0.85
        self.l_free = -0.4
        self.l_min = -5.0
        self.l_max = 5.0

        # Downsample laser beams to keep CPU low
        self.beam_step = 2   # use every 2nd ray (increase to 4 if slow)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ROS interfaces
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 1)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.timer = self.create_timer(1.0, self.publish_map)  # 1 Hz

        self.last_scan = None
        self.get_logger().info("OccupancyGridMapper running. Drive the robot to build a map.")

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def lookup_pose_odom_of(self, source_frame: str):
        """
        Returns (x, y, yaw) of source_frame in odom frame using latest available TF.
        """
        tf = self.tf_buffer.lookup_transform("odom", source_frame, Time())
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        return tx, ty, yaw

    def on_scan(self, scan: LaserScan):
        self.last_scan = scan

        # Use the scan frame directly (yours is base_scan)
        scan_frame = scan.header.frame_id

        try:
            rx, ry, ryaw = self.lookup_pose_odom_of(scan_frame)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed odom->{scan_frame}: {e}")
            return

        r_gx, r_gy = self.world_to_grid(rx, ry)
        if not self.in_bounds(r_gx, r_gy):
            return

        angle = scan.angle_min
        for i, r in enumerate(scan.ranges):
            # downsample rays
            if i % self.beam_step != 0:
                angle += scan.angle_increment
                continue

            if math.isinf(r) or math.isnan(r):
                angle += scan.angle_increment
                continue

            if r < scan.range_min or r > scan.range_max:
                angle += scan.angle_increment
                continue

            theta = ryaw + angle
            ex = rx + r * math.cos(theta)
            ey = ry + r * math.sin(theta)

            e_gx, e_gy = self.world_to_grid(ex, ey)
            if not self.in_bounds(e_gx, e_gy):
                angle += scan.angle_increment
                continue

            line = bresenham(r_gx, r_gy, e_gx, e_gy)
            if len(line) < 2:
                angle += scan.angle_increment
                continue

            # free cells (excluding endpoint)
            for gx, gy in line[:-1]:
                if self.in_bounds(gx, gy):
                    self.log_odds[gy, gx] = np.clip(
                        self.log_odds[gy, gx] + self.l_free, self.l_min, self.l_max
                    )

            # occupied endpoint
            gx, gy = line[-1]
            if self.in_bounds(gx, gy):
                self.log_odds[gy, gx] = np.clip(
                    self.log_odds[gy, gx] + self.l_occ, self.l_min, self.l_max
                )

            angle += scan.angle_increment

    def publish_map(self):
        if self.last_scan is None:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        msg.info.resolution = float(self.resolution)
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.orientation.w = 1.0

        # log-odds -> probability -> [0..100]
        p_occ = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        occ = (p_occ * 100.0).astype(np.int16)

        # unknown if close to 0 log-odds
        unknown = np.abs(self.log_odds) < 0.05
        occ[unknown] = -1

        msg.data = occ.astype(np.int8).flatten(order="C").tolist()
        self.map_pub.publish(msg)


def main():
    rclpy.init()
    node = OccupancyGridMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
