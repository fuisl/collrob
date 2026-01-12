import math
import random
import heapq
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import tf2_ros


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class SimpleExplorer(Node):
    """
    Frontier exploration + A* planning + simple path following.
    Inputs: /map, TF(map->base_footprint), /scan (safety)
    Output: /cmd_vel
    """

    def __init__(self):
        super().__init__("simple_explorer")

        # ---- Frames/topics ----
        self.map_frame = "map"
        self.base_frame = "base_footprint"
        self.map_topic = "/map"
        self.scan_topic = "/scan"
        self.cmd_vel_topic = "/cmd_vel"

        # ---- Map / planning params ----
        self.free_thresh = 20           # 0..100 considered free if <= this
        self.goal_replan_period = 2.0   # seconds
        self.frontier_sample_cap = 50000 # cap frontier points for speed

        # A* neighborhood: 8-connected
        self.neigh = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                      (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                      (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]

        # ---- Controller params ----
        self.lin_speed = 0.08
        self.ang_speed = 0.5
        self.yaw_turn_thresh = 0.45      # rad: rotate-in-place if |err| above this
        self.waypoint_tol = 0.15         # m
        self.lookahead_idx = 3           # follow a point a few steps ahead

        # ---- Safety from scan ----
        self.use_scan_safety = True
        self.front_arc_deg = 25.0
        self.stop_dist = 0.28
        self.recovery_turn_time = 1.2
        self.recovering = False
        self.recovery_until = 0.0
        self.recovery_dir = 1.0

        # ---- State ----
        self.map_msg = None
        self.grid = None
        self.info = None
        self.last_scan = None

        self.path_world = []   # list of (x,y) in map frame
        self.path_i = 0

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- QoS ----
        # Map in Nav2/SLAM is usually transient-local; match it to receive latched map.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, scan_qos)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.plan_timer = self.create_timer(self.goal_replan_period, self.plan_tick)
        self.ctrl_timer = self.create_timer(0.05, self.control_tick)  # 20 Hz
        
        self.path_marker_pub = self.create_publisher(Marker, "/explorer/a_star_path", 1)

        self.get_logger().info("SimpleExplorer running. It will pick frontiers and drive via /cmd_vel.")

    # ---------- Callbacks ----------
    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.info = msg.info
        data = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        self.grid = data
        
        self.get_logger().info(f"map stats: min={self.grid.min()} max={self.grid.max()} known={np.mean(self.grid!=-1):.3f}")

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    # ---------- TF / pose ----------
    def get_robot_pose_map(self):
        tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        return x, y, yaw

    # ---------- Map helpers ----------
    def world_to_grid(self, x, y):
        res = self.info.resolution
        ox = self.info.origin.position.x
        oy = self.info.origin.position.y
        gx = int((x - ox) / res)
        gy = int((y - oy) / res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        res = self.info.resolution
        ox = self.info.origin.position.x
        oy = self.info.origin.position.y
        x = ox + (gx + 0.5) * res
        y = oy + (gy + 0.5) * res
        return x, y

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.info.width and 0 <= gy < self.info.height

    def is_free(self, gx, gy):
        v = int(self.grid[gy, gx])
        # unknown is -1
        if v < 0:
            return False
        # free if occupancy prob < 50%
        return v < 50


    # ---------- Frontier detection ----------
    def compute_frontier_mask(self):
        """Frontier = unknown (-1) adjacent (4-neigh) to free cell (no wrap-around)."""
        unknown = (self.grid == -1)
        free = (self.grid >= 0) & (self.grid < 50)


        frontier = np.zeros_like(unknown, dtype=bool)

        # up/down neighbors (no wrap)
        frontier[1:, :] |= unknown[1:, :] & free[:-1, :]
        frontier[:-1, :] |= unknown[:-1, :] & free[1:, :]

        # left/right neighbors (no wrap)
        frontier[:, 1:] |= unknown[:, 1:] & free[:, :-1]
        frontier[:, :-1] |= unknown[:, :-1] & free[:, 1:]

        return frontier


    def pick_goal_near_frontier(self):
        """
        Returns a reachable goal cell (gx,gy) in FREE space adjacent to a frontier cluster.
        Strategy:
          - cluster frontier cells
          - choose a cluster (largest or farthest)
          - pick a nearby FREE neighbor cell as goal
        """
        frontier = self.compute_frontier_mask()
        frontier_count = int(np.sum(frontier))
        self.get_logger().info(f"frontier_count={frontier_count}")

        ys, xs = np.where(frontier)
        if len(xs) == 0:
            return None

        # cap for speed
        if len(xs) > self.frontier_sample_cap:
            idx = np.random.choice(len(xs), self.frontier_sample_cap, replace=False)
            xs = xs[idx]
            ys = ys[idx]

        frontier_set = set(zip(xs.tolist(), ys.tolist()))
        visited = set()

        # robot position (grid)
        rx, ry, _ = self.get_robot_pose_map()
        rgx, rgy = self.world_to_grid(rx, ry)

        best = None
        best_score = -1e18

        for (sx, sy) in list(frontier_set):
            if (sx, sy) in visited:
                continue

            # BFS cluster
            q = deque([(sx, sy)])
            visited.add((sx, sy))
            cluster = []
            free_neighbors = []

            while q:
                x, y = q.popleft()
                cluster.append((x, y))

                # collect adjacent free cells (reachable goal candidates)
                for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                    nx, ny = x + dx, y + dy
                    if self.in_bounds(nx, ny) and self.is_free(nx, ny):
                        free_neighbors.append((nx, ny))

                # expand cluster (4-neigh on frontier)
                for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                    nx, ny = x + dx, y + dy
                    if (nx, ny) in frontier_set and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        q.append((nx, ny))

            if len(cluster) < 3 or len(free_neighbors) == 0:
                continue

            # cluster centroid (grid)
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)

            # pick a free neighbor closest to centroid
            goal = min(free_neighbors, key=lambda p: (p[0]-cx)**2 + (p[1]-cy)**2)

            # score: prefer larger clusters and farther from robot (push outward)
            dist2 = (goal[0]-rgx)**2 + (goal[1]-rgy)**2
            score = 0.7 * len(cluster) + 0.3 * math.sqrt(dist2)

            if score > best_score:
                best_score = score
                best = goal

        return best

    # ---------- A* ----------
    def astar(self, start, goal):
        sx, sy = start
        gx, gy = goal

        if not self.in_bounds(sx, sy) or not self.in_bounds(gx, gy):
            return None
        if not self.is_free(sx, sy) or not self.is_free(gx, gy):
            return None

        def h(x, y):
            return math.hypot(gx - x, gy - y)

        openpq = []
        heapq.heappush(openpq, (h(sx, sy), 0.0, (sx, sy)))
        came = { (sx, sy): None }
        gscore = { (sx, sy): 0.0 }

        while openpq:
            _, gc, (x, y) = heapq.heappop(openpq)
            if (x, y) == (gx, gy):
                # reconstruct
                path = []
                cur = (x, y)
                while cur is not None:
                    path.append(cur)
                    cur = came[cur]
                path.reverse()
                return path

            # stale entry
            if gc > gscore.get((x, y), 1e18):
                continue

            for dx, dy, cost in self.neigh:
                nx, ny = x + dx, y + dy
                if not self.in_bounds(nx, ny):
                    continue
                if not self.is_free(nx, ny):
                    continue

                ng = gc + cost
                if ng < gscore.get((nx, ny), 1e18):
                    gscore[(nx, ny)] = ng
                    came[(nx, ny)] = (x, y)
                    heapq.heappush(openpq, (ng + h(nx, ny), ng, (nx, ny)))

        return None

    # ---------- Planning loop ----------
    def plan_tick(self):
        if self.grid is None or self.info is None:
            self.get_logger().warn("Waiting for /map ...")
            return

        if not self.tf_buffer.can_transform(self.map_frame, self.base_frame, Time()):
            self.get_logger().warn("Waiting for TF map->base_footprint ...")
            return

        # If we still have a path, keep it unless we reached end.
        if self.path_world and self.path_i < len(self.path_world):
            return

        try:
            rx, ry, _ = self.get_robot_pose_map()
        except Exception as e:
            self.get_logger().warn(f"TF map->{self.base_frame} not ready: {e}")
            return

        start = self.world_to_grid(rx, ry)

        goal = self.pick_goal_near_frontier()
        if goal is None:
            known = float(np.mean(self.grid != -1))
            if known < 0.02:
                self.get_logger().info("No frontiers yet (map still mostly unknown). Keep moving / wait for mapping.")
            else:
                self.get_logger().info("No frontiers found. Exploration may be complete.")
            return


        path = self.astar(start, goal)
        if path is None or len(path) < 2:
            # blacklist behavior could be added; for now just try next tick
            self.get_logger().warn("A* failed to find a path to chosen frontier. Will retry.")
            return
        self.path_world = [self.grid_to_world(px, py) for (px, py) in path]
        self.path_i = 0
        
        # publish path marker
        self.publish_path_marker()

        self.get_logger().info(f"Planned path: {len(self.path_world)} waypoints to goal cell {goal}")

    # ---------- Control loop ----------
    def control_tick(self):
        # Recovery behavior (turn in place if too close)
        now = self.get_clock().now().nanoseconds / 1e9
        if self.recovering:
            if now >= self.recovery_until:
                self.recovering = False
            else:
                tw = Twist()
                tw.angular.z = float(self.recovery_dir * self.ang_speed)
                self.cmd_pub.publish(tw)
                return

        # Safety stop if obstacle ahead
        if self.use_scan_safety and self.last_scan is not None:
            if self.too_close_ahead(self.last_scan):
                self.recovering = True
                self.recovery_until = now + self.recovery_turn_time
                self.recovery_dir = 1.0 if random.random() < 0.5 else -1.0
                self.cmd_pub.publish(Twist())  # stop
                return

        if not self.path_world or self.path_i >= len(self.path_world):
            self.cmd_pub.publish(Twist())
            return

        try:
            rx, ry, ryaw = self.get_robot_pose_map()
        except Exception:
            self.cmd_pub.publish(Twist())
            return

        # choose a lookahead target on the path
        idx = min(self.path_i + self.lookahead_idx, len(self.path_world) - 1)
        tx, ty = self.path_world[idx]

        dx = tx - rx
        dy = ty - ry
        dist = math.hypot(dx, dy)
        ang = math.atan2(dy, dx)
        err = wrap_pi(ang - ryaw)

        # advance path index if close to current waypoint
        cx, cy = self.path_world[self.path_i]
        if math.hypot(cx - rx, cy - ry) < self.waypoint_tol:
            self.path_i += 1

        cmd = Twist()

        # rotate-in-place if heading error is large
        if abs(err) > self.yaw_turn_thresh:
            cmd.angular.z = float(self.ang_speed * (1.0 if err > 0 else -1.0))
        else:
            # move forward and steer
            cmd.linear.x = float(self.lin_speed * max(0.0, 1.0 - abs(err) / self.yaw_turn_thresh))
            cmd.angular.z = float(0.8 * self.ang_speed * err)

        self.cmd_pub.publish(cmd)

    def too_close_ahead(self, scan: LaserScan):
        # check a frontal arc for minimum range
        if not scan.ranges:
            return False

        arc = math.radians(self.front_arc_deg)
        # scan angles: angle_min ... angle_max
        min_r = 1e9
        for i, r in enumerate(scan.ranges):
            a = scan.angle_min + i * scan.angle_increment
            if abs(a) > arc:
                continue
            if math.isinf(r) or math.isnan(r):
                continue
            if r < min_r:
                min_r = r
        return min_r < self.stop_dist
    
    def publish_path_marker(self):
        if not self.path_world:
            return

        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()

        m.ns = "a_star"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD

        # thickness (meters)
        m.scale.x = 0.03

        # color (RGBA) - choose any
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 1.0

        m.points = []
        for (x, y) in self.path_world:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.05
            m.points.append(p)

        self.path_marker_pub.publish(m)



def main():
    rclpy.init()
    node = SimpleExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
