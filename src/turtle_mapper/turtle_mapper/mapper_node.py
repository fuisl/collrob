import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped

import tf2_ros


def yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def se2_compose(a, b):
    """a=(x,y,th), b=(dx,dy,dth) in frame of a; returns a ⊕ b"""
    ax, ay, ath = a
    bx, by, bth = b
    c = math.cos(ath)
    s = math.sin(ath)
    x = ax + c * bx - s * by
    y = ay + s * bx + c * by
    th = wrap_pi(ath + bth)
    return (x, y, th)


def se2_inverse(a):
    x, y, th = a
    c = math.cos(th)
    s = math.sin(th)
    # inverse transform
    ix = -(c * x + s * y)
    iy = -(-s * x + c * y)
    ith = wrap_pi(-th)
    return (ix, iy, ith)


def se2_between(a, b):
    """returns delta such that a ⊕ delta = b. delta is expressed in frame of a."""
    return se2_compose(se2_inverse(a), b)


def bresenham(x0, y0, x1, y1):
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


class SlamMapper(Node):
    def __init__(self):
        super().__init__("slam_mapper")

        # Frames from your TF tree
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.base_frame = "base_footprint"   # planar pose
        self.scan_frame = "base_scan"        # scan comes from here

        # Map params
        self.resolution = 0.05
        self.size_x_m = 40.0
        self.size_y_m = 40.0
        self.width = int(self.size_x_m / self.resolution)
        self.height = int(self.size_y_m / self.resolution)
        self.origin_x = -self.size_x_m / 2.0
        self.origin_y = -self.size_y_m / 2.0

        # Log-odds
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.l_occ = 0.85
        self.l_free = -0.4
        self.l_min = -5.0
        self.l_max = 5.0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ROS I/O
        map_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.timer = self.create_timer(1.0, self.publish_map)

        # SLAM state
        self.prev_odom_pose = None              # (x,y,yaw) in odom
        self.est_map_pose = (0.0, 0.0, 0.0)     # (x,y,yaw) in map
        self.initialized = False

        # Performance knobs
        self.beam_step_map = 2    # mapping rays
        self.beam_step_match = 6  # scan-matching rays (downsample more)

        # Scan matching search window (start small)
        self.search_xy = 0.10     # +/- meters
        self.search_th = math.radians(6.0)   # +/- radians
        self.step_xy = 0.02
        self.step_th = math.radians(1.0)

        self.get_logger().info("SLAM Mapper started (scan-matching SLAM-lite).")

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def lookup_se2(self, target, source):
        tf = self.tf_buffer.lookup_transform(target, source, Time())
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        return (tx, ty, yaw)

    def on_scan(self, scan: LaserScan):
        # 1) Read odom pose of base_footprint
        try:
            odom_pose = self.lookup_se2(self.odom_frame, self.base_frame)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed {self.odom_frame}->{self.base_frame}: {e}")
            return

        # Initialize: map frame coincides with odom at start
        if not self.initialized:
            self.prev_odom_pose = odom_pose
            self.est_map_pose = odom_pose  # start map pose = odom pose
            self.initialized = True
            self.broadcast_map_to_odom(odom_pose, self.est_map_pose)
            return

        # 2) Predict using odom delta (in base frame of previous step)
        delta = se2_between(self.prev_odom_pose, odom_pose)
        pred_pose = se2_compose(self.est_map_pose, delta)

        # 3) Correct with scan matching (once map has some structure)
        corr_pose = self.scan_match(pred_pose, scan)

        # 4) Update state
        self.prev_odom_pose = odom_pose
        self.est_map_pose = corr_pose

        # 5) Broadcast map->odom for the rest of the system
        self.broadcast_map_to_odom(odom_pose, self.est_map_pose)

        # 6) Update occupancy grid using corrected pose, but use scan angles from LaserScan
        self.integrate_scan(self.est_map_pose, scan)

    def known_fraction(self):
        # how much of map has been updated from 0 log-odds
        return float(np.mean(np.abs(self.log_odds) > 0.05))

    def scan_match(self, pred_pose, scan: LaserScan):
        # If map is still mostly unknown, scan matching is meaningless.
        if self.known_fraction() < 0.02:
            return pred_pose

        best_pose = pred_pose
        best_score = -1e18

        # Precompute angles for matching (downsample)
        indices = range(0, len(scan.ranges), self.beam_step_match)

        # Candidate search
        th0 = pred_pose[2]
        for dth in np.arange(-self.search_th, self.search_th + 1e-9, self.step_th):
            th = wrap_pi(th0 + float(dth))
            c = math.cos(th)
            s = math.sin(th)

            for dx in np.arange(-self.search_xy, self.search_xy + 1e-9, self.step_xy):
                for dy in np.arange(-self.search_xy, self.search_xy + 1e-9, self.step_xy):
                    x = pred_pose[0] + float(dx)
                    y = pred_pose[1] + float(dy)

                    score = 0.0
                    angle = scan.angle_min

                    # score using endpoints landing on occupied cells (log_odds high)
                    for i in indices:
                        r = scan.ranges[i]
                        if math.isinf(r) or math.isnan(r):
                            angle += scan.angle_increment * self.beam_step_match
                            continue
                        if r < scan.range_min or r > scan.range_max:
                            angle += scan.angle_increment * self.beam_step_match
                            continue

                        a = th + (scan.angle_min + i * scan.angle_increment)
                        ex = x + r * math.cos(a)
                        ey = y + r * math.sin(a)
                        gx, gy = self.world_to_grid(ex, ey)
                        if self.in_bounds(gx, gy):
                            # reward “more occupied”
                            score += float(self.log_odds[gy, gx])

                        angle += scan.angle_increment * self.beam_step_match

                    if score > best_score:
                        best_score = score
                        best_pose = (x, y, th)

        return best_pose

    def integrate_scan(self, pose, scan: LaserScan):
        rx, ry, ryaw = pose
        r_gx, r_gy = self.world_to_grid(rx, ry)
        if not self.in_bounds(r_gx, r_gy):
            return

        free_cap = scan.range_max * 0.98

        for i in range(0, len(scan.ranges), self.beam_step_map):
            r_raw = scan.ranges[i]

            # Decide whether this ray ends in an obstacle
            hit = True
            if math.isnan(r_raw):
                continue

            if math.isinf(r_raw):
                # no hit: carve free space out to cap
                r = free_cap
                hit = False
            else:
                r = r_raw
                if r < scan.range_min:
                    continue
                if r > scan.range_max:
                    # treat as no hit (rare but safe)
                    r = free_cap
                    hit = False

            angle = scan.angle_min + i * scan.angle_increment
            theta = ryaw + angle

            ex = rx + r * math.cos(theta)
            ey = ry + r * math.sin(theta)

            e_gx, e_gy = self.world_to_grid(ex, ey)
            if not self.in_bounds(e_gx, e_gy):
                continue

            line = bresenham(r_gx, r_gy, e_gx, e_gy)
            if len(line) < 2:
                continue

            # Free along the ray (exclude endpoint if it's a hit)
            free_cells = line[:-1] if hit else line
            for gx, gy in free_cells:
                if self.in_bounds(gx, gy):
                    self.log_odds[gy, gx] = np.clip(
                        self.log_odds[gy, gx] + self.l_free,
                        self.l_min, self.l_max
                    )

            # Occupied only if we had a real hit
            if hit:
                gx, gy = line[-1]
                if self.in_bounds(gx, gy):
                    self.log_odds[gy, gx] = np.clip(
                        self.log_odds[gy, gx] + self.l_occ,
                        self.l_min, self.l_max
                    )


    def broadcast_map_to_odom(self, odom_pose, map_pose):
        # T_map_odom = T_map_base * inv(T_odom_base)
        inv_odom = se2_inverse(odom_pose)
        map_to_odom = se2_compose(map_pose, inv_odom)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame

        t.transform.translation.x = float(map_to_odom[0])
        t.transform.translation.y = float(map_to_odom[1])
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = quat_from_yaw(map_to_odom[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def publish_map(self):
        if not self.initialized:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.info.resolution = float(self.resolution)
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.orientation.w = 1.0

        p_occ = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        occ = (p_occ * 100.0).astype(np.int16)
        unknown = np.abs(self.log_odds) < 0.05
        occ[unknown] = -1

        msg.data = occ.astype(np.int8).flatten(order="C").tolist()
        self.map_pub.publish(msg)


def main():
    rclpy.init()
    node = SlamMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
