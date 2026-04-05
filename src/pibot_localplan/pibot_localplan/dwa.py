#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException

import numpy as np
import math
from enum import Enum

from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from obstacle_detector.msg import Obstacles
from pibot_msg.action import AcoPlan
from rclpy.action import ActionClient



class PlannerState(Enum):
    IDLE       = 0
    ROTATING   = 1
    FOLLOWING  = 2
    REPLANNING = 3


class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner_node')

        self.declare_parameter('max_speed',      0.15)
        self.declare_parameter('min_speed',      0.0)
        self.declare_parameter('max_yaw_rate',   0.6)
        self.declare_parameter('min_yaw_rate',   0.2)
        self.declare_parameter('max_accel',      1.0)
        self.declare_parameter('max_dyaw_rate',  1.0)
        self.declare_parameter('v_res',          0.02)
        self.declare_parameter('w_res',          0.1)
        self.declare_parameter('dt',             0.1)
        self.declare_parameter('predict_time',   3.5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('yaw_goal_tolerance', 0.4)
        self.declare_parameter('rotation_kp',    1.5)
        self.declare_parameter('lethal_cost',    90)
        self.declare_parameter('alpha_heading',  0.3)
        self.declare_parameter('beta_dist',      1.0)
        self.declare_parameter('gamma_vel',      0.5)
        self.declare_parameter('delta_ghead',    0.0)
        self.declare_parameter('global_frame',   'map')
        self.declare_parameter('robot_frame',    'base_footprint')
        self.declare_parameter('replan_enabled',        True)
        self.declare_parameter('replan_deviation_dist', 0.5)
        self.declare_parameter('replan_cooldown',       5.0)
        self.declare_parameter('stuck_time_threshold',  3.0)
        self.declare_parameter('stuck_dist_threshold',  0.05)
        self.declare_parameter('robot_radius',   0.20)
        self.declare_parameter('c_safe',         0.1)
        self.declare_parameter('k_dynamic',      3.0)
        self.declare_parameter('epsilon_dynobs', 0.4)
        self.declare_parameter('dynobs_lethal',  0.0)
        self.declare_parameter('dynobs_range',   2.5)

        self.max_v       = self.get_parameter('max_speed').value
        self.min_v       = self.get_parameter('min_speed').value
        self.max_w       = self.get_parameter('max_yaw_rate').value
        self.min_w       = self.get_parameter('min_yaw_rate').value
        self.max_accel   = self.get_parameter('max_accel').value
        self.max_dyaw    = self.get_parameter('max_dyaw_rate').value
        self.v_res       = self.get_parameter('v_res').value
        self.w_res       = self.get_parameter('w_res').value
        self.dt          = self.get_parameter('dt').value
        self.predict_time= self.get_parameter('predict_time').value
        self.goal_tol    = self.get_parameter('goal_tolerance').value
        self.yaw_tol     = self.get_parameter('yaw_goal_tolerance').value
        self.rot_kp      = self.get_parameter('rotation_kp').value
        self.lethal_cost = self.get_parameter('lethal_cost').value
        self.p_head      = self.get_parameter('alpha_heading').value
        self.p_dist      = self.get_parameter('beta_dist').value
        self.p_vel       = self.get_parameter('gamma_vel').value
        self.p_ghead     = self.get_parameter('delta_ghead').value
        self.global_frame        = self.get_parameter('global_frame').value
        self.robot_frame         = self.get_parameter('robot_frame').value
        self.replan_enabled      = self.get_parameter('replan_enabled').value
        self.replan_deviation_dist = self.get_parameter('replan_deviation_dist').value
        self.replan_cooldown     = self.get_parameter('replan_cooldown').value
        self.stuck_time_threshold= self.get_parameter('stuck_time_threshold').value
        self.stuck_dist_threshold= self.get_parameter('stuck_dist_threshold').value
        self.R_robot       = self.get_parameter('robot_radius').value
        self.c_safe        = self.get_parameter('c_safe').value
        self.k_dynamic     = self.get_parameter('k_dynamic').value
        self.p_dynobs      = self.get_parameter('epsilon_dynobs').value
        self.dynobs_lethal = self.get_parameter('dynobs_lethal').value
        self.dynobs_range  = self.get_parameter('dynobs_range').value

        self.robot_state    = np.zeros(5)
        self.waypoints      = []
        self.smooth_path    = []
        self.final_goal_pose= None
        self.state          = PlannerState.IDLE
        self.current_wp_idx = 0
        self.costmap_np     = None
        self.costmap_info   = None
        self.num_steps  = int(self.predict_time / self.dt)
        self.time_steps = np.arange(1, self.num_steps + 1) * self.dt
        self.is_replanning       = False
        self._replan_goal_handle = None
        self.last_replan_time    = 0.0
        self.last_moved_time        = 0.0
        self.last_check_pos         = np.array([0.0, 0.0])
        self.standstill_initialized = False
        self.dynamic_circles = []

        self._action_client  = ActionClient(self, AcoPlan, 'aco_plan')
        self.cmd_vel_pub     = self.create_publisher(Twist, '/pibot_controller/cmd_vel_unstamped', 10)
        self.local_goal_pub  = self.create_publisher(PoseStamped, '/local_goal', 10)
        self.traj_pub        = self.create_publisher(MarkerArray, '/visual_paths', 10)

        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10,
                                durability=DurabilityPolicy.VOLATILE)
        qos_map = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.odom_sub     = self.create_subscription(Odometry, '/odometry/filtered', self.velocity_callback, qos_sensor)
        self.costmap_sub  = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, qos_map)
        self.wp_sub       = self.create_subscription(Path, '/improve_aco3/waypoints', self.waypoints_callback, 10)
        self.smooth_sub   = self.create_subscription(Path, '/improve_aco3/path', self.smooth_path_callback, 10)
        self.dynobs_sub   = self.create_subscription(Obstacles, '/dynamic_obstacles', self.dynamic_obs_callback, 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"DWA Planner Started | R_robot={self.R_robot}m, c_safe={self.c_safe}m, k_dyn={self.k_dynamic}, ε_dyn={self.p_dynobs}")

    # ================================================================
    # ODOMETRY / TF
    # ================================================================

    def velocity_callback(self, msg: Odometry):
        self.robot_state[3] = msg.twist.twist.linear.x
        self.robot_state[4] = msg.twist.twist.angular.z

    def update_pose_from_tf(self) -> bool:
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, rclpy.time.Time())
            self.robot_state[0] = t.transform.translation.x
            self.robot_state[1] = t.transform.translation.y
            q = t.transform.rotation
            self.robot_state[2] = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            return True
        except TransformException:
            return False

    # ================================================================
    # COSTMAP
    # ================================================================

    def costmap_callback(self, msg: OccupancyGrid):
        if self.costmap_info is None:
            self.get_logger().info(f"Map Received: {msg.info.width}x{msg.info.height}")
        self.costmap_info = msg.info
        raw = np.array(msg.data, dtype=np.int8)
        self.costmap_np = raw.reshape((msg.info.height, msg.info.width))

    # ================================================================
    # PATH CALLBACKS
    # ================================================================

    def waypoints_callback(self, msg: Path):
        self.waypoints = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        if not self.waypoints:
            return
        self.final_goal_pose = msg.poses[-1]
        self.current_wp_idx  = 0
        self.is_replanning   = False
        self._reset_standstill()
        if self.state in [PlannerState.IDLE, PlannerState.FOLLOWING, PlannerState.REPLANNING]:
            next_wp = self.waypoints[1] if len(self.waypoints) > 1 else self.waypoints[0]
            yaw_err = self._compute_yaw_error_to(next_wp)
            if yaw_err < self.yaw_tol * 3.0:
                self.state = PlannerState.FOLLOWING
                self.get_logger().info(f"Path received, yaw_err={math.degrees(yaw_err):.1f}° -> FOLLOWING")
            else:
                self.state = PlannerState.ROTATING
                self.get_logger().info(f"Path received, yaw_err={math.degrees(yaw_err):.1f}° -> ROTATING")

    def smooth_path_callback(self, msg: Path):
        if msg.poses:
            self.smooth_path = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

    # ================================================================
    # DYNAMIC OBSTACLE CALLBACK
    # ================================================================

    def dynamic_obs_callback(self, msg: Obstacles):
        """Nhận dữ liệu từ /dynamic_obstacles, lấy velocity từ Kalman tracker."""
        self.dynamic_circles = [
            (c.center.x, c.center.y, c.true_radius, c.velocity.x, c.velocity.y)
            for c in msg.circles
        ]

    # ================================================================
    # OBSTACLE TRAJECTORY PREDICTION
    # ================================================================

    def _predict_obs_trajectory(self, cx: float, cy: float, vx: float, vy: float) -> tuple:
        xs_obs = cx + vx * self.time_steps
        ys_obs = cy + vy * self.time_steps
        return xs_obs, ys_obs

    # ================================================================
    # DIST_DYNAMIC
    # ================================================================

    def compute_dist_dynamic(self, xs: np.ndarray, ys: np.ndarray) -> float:
        """
        Tính dist_dynamic giữa trajectory robot và quỹ đạo dự đoán của obstacle.

        Công thức (theo slide):
            d(k)     = |P_robot(t+k) - P_obs(t+k)|
            Δd(k)    = d(k) - (R_robot + R_obs + c_safe)
            raw      = 1 - exp(-k_dyn * min_k(Δd(k)))
            dist_dyn = clip(raw, 0.0, 1.0)  ∈ [0, 1]
        """
        if not self.dynamic_circles:
            return 1.0

        worst = 1.0
        for (cx, cy, R_obs, vx, vy) in self.dynamic_circles:
            xs_obs, ys_obs = self._predict_obs_trajectory(cx, cy, vx, vy)
            dx = xs - xs_obs
            dy = ys - ys_obs
            d  = np.sqrt(dx*dx + dy*dy)

            if float(np.min(d)) > self.dynobs_range:
                continue

            delta_d    = d - (self.R_robot + R_obs + self.c_safe)
            dist_steps = np.clip(1.0 - np.exp(-self.k_dynamic * delta_d), 0.0, 1.0)

            obs_worst = float(np.min(dist_steps))
            if obs_worst < worst:
                worst = obs_worst
        return worst

    # ================================================================
    # STANDSTILL DETECTION
    # ================================================================

    def _reset_standstill(self):
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_moved_time        = now
        self.last_check_pos         = np.array([self.robot_state[0], self.robot_state[1]])
        self.standstill_initialized = True

    def check_standstill(self) -> bool:
        if not self.standstill_initialized:
            self._reset_standstill()
            return False
        now        = self.get_clock().now().nanoseconds / 1e9
        cur_pos    = np.array([self.robot_state[0], self.robot_state[1]])
        dist_moved = float(np.linalg.norm(cur_pos - self.last_check_pos))
        if dist_moved >= self.stuck_dist_threshold:
            self.last_moved_time = now
            self.last_check_pos  = cur_pos
            return False
        standstill_duration = now - self.last_moved_time
        if standstill_duration >= self.stuck_time_threshold:
            self.get_logger().warn(f"⚠️ Standstill: robot không di chuyển {standstill_duration:.1f}s (dist={dist_moved:.3f}m)")
            return True
        return False

    # ================================================================
    # REPLAN
    # ================================================================

    def _replan_on_cooldown(self) -> bool:
        now = self.get_clock().now().nanoseconds / 1e9
        return (now - self.last_replan_time) < self.replan_cooldown

    def check_deviation(self) -> bool:
        if self.smooth_path is None or len(self.smooth_path) == 0:
            return False
        rx, ry = self.robot_state[0], self.robot_state[1]
        dists  = np.hypot(self.smooth_path[:, 0] - rx, self.smooth_path[:, 1] - ry)
        min_d  = float(np.min(dists))
        if min_d > self.replan_deviation_dist:
            self.get_logger().warn(f"⚠️ Deviation: {min_d:.2f}m > {self.replan_deviation_dist}m")
            return True
        return False

    def trigger_replan(self, reason: str = ''):
        if not self.replan_enabled:      return
        if self.is_replanning:           return
        if self._replan_on_cooldown():   return
        if self.final_goal_pose is None: return
        if not self._action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("ACO action server not available, skip replan.")
            return
        self.is_replanning    = True
        self.last_replan_time = self.get_clock().now().nanoseconds / 1e9
        self._reset_standstill()
        self.state = PlannerState.REPLANNING
        self.get_logger().info(f"🔄 Triggering replan [{reason}]...")
        goal_msg = AcoPlan.Goal()
        goal_msg.goal_pose = self.final_goal_pose
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._replan_feedback_callback)
        future.add_done_callback(self._replan_goal_response_callback)

    def _replan_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().debug(f"Replan: iter={fb.iteration}, best={fb.current_best_length:.2f}")

    def _replan_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Replan goal REJECTED.")
            self.is_replanning = False
            self.state = PlannerState.FOLLOWING
            return
        self._replan_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._replan_result_callback)

    def _replan_result_callback(self, future):
        self.is_replanning       = False
        self._replan_goal_handle = None
        try:
            result = future.result().result
            if result.path and len(result.path.poses) > 0:
                self.get_logger().info(f"✅ Replan success! length={result.total_length:.2f}")
            else:
                self.get_logger().warn("⚠️ Replan returned empty path.")
                self.state = PlannerState.FOLLOWING
        except Exception as e:
            self.get_logger().error(f"Replan error: {e}")
            self.state = PlannerState.FOLLOWING

    def cancel_replan(self):
        if self._replan_goal_handle is not None:
            self._replan_goal_handle.cancel_goal_async()
            self._replan_goal_handle = None
        self.is_replanning = False

    # ================================================================
    # HELPERS
    # ================================================================

    def normalize_angle(self, angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _compute_yaw_error_to(self, target) -> float:
        rx, ry, ryaw = self.robot_state[0], self.robot_state[1], self.robot_state[2]
        return abs(self.normalize_angle(math.atan2(target[1]-ry, target[0]-rx) - ryaw))

    def get_current_waypoint(self):
        if not self.waypoints or self.current_wp_idx >= len(self.waypoints):
            return None
        return self.waypoints[self.current_wp_idx]

    # ================================================================
    # WAYPOINT PROGRESS
    # ================================================================

    def update_waypoint_progress(self):
        if not self.waypoints:
            return
        while self.current_wp_idx < len(self.waypoints) - 1:
            wp     = self.waypoints[self.current_wp_idx]
            rx, ry = self.robot_state[0], self.robot_state[1]
            dist   = math.hypot(rx - wp[0], ry - wp[1])
            wp_tol = self.goal_tol * 2.0
            next_wp   = self.waypoints[self.current_wp_idx + 1]
            seg_x, seg_y = next_wp[0] - wp[0], next_wp[1] - wp[1]
            rob_x, rob_y = rx - wp[0], ry - wp[1]
            dot        = seg_x * rob_x + seg_y * rob_y
            seg_len_sq = seg_x**2 + seg_y**2
            t = (dot / seg_len_sq) if seg_len_sq > 0 else 0.0
            if dist < wp_tol or t > 0.0:
                self.get_logger().info(f"WP {self.current_wp_idx} -> {self.current_wp_idx+1}/{len(self.waypoints)-1} (dist={dist:.2f}m, t={t:.2f})")
                self.current_wp_idx += 1
            else:
                break

    # ================================================================
    # COSTMAP COLLISION CHECK
    # ================================================================

    def check_trajectory_collision_batch(self, xs: np.ndarray, ys: np.ndarray) -> int:
        if self.costmap_np is None or self.costmap_info is None:
            return 100
        res = self.costmap_info.resolution
        ox  = self.costmap_info.origin.position.x
        oy  = self.costmap_info.origin.position.y
        w   = self.costmap_info.width
        h   = self.costmap_info.height
        mx  = ((xs - ox) / res).astype(int)
        my  = ((ys - oy) / res).astype(int)
        if not np.all((mx >= 0) & (my >= 0) & (mx < w) & (my < h)):
            return 100
        costs = self.costmap_np[my, mx]
        costs = np.where(costs == -1, 100, costs)
        return int(np.max(costs))

    # ================================================================
    # VISUALIZATION (trajectories only)
    # ================================================================

    def clear_trajectories(self):
        """Xóa tất cả visualization trajectories từ RViz"""
        marker_array = MarkerArray()
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        marker_array.markers.append(del_m)
        self.traj_pub.publish(marker_array)

    def visualize_trajectories(self, all_paths, best_idx: int):
        marker_array = MarkerArray()
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        marker_array.markers.append(del_m)
        step = 3
        for i, (xs, ys) in enumerate(all_paths):
            if i != best_idx and i % step != 0:
                continue
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp    = self.get_clock().now().to_msg()
            m.type            = Marker.LINE_STRIP
            m.action          = Marker.ADD
            m.pose.orientation.w = 1.0
            if i == best_idx:
                m.ns = 'dwa_best'; m.id = 9999; m.scale.x = 0.02
                m.color.r = 1.0;    m.color.a = 1.0
            else:
                m.ns = 'dwa_paths'; m.id = i; m.scale.x = 0.02
                m.color.r = 1.0;    m.color.a = 1.0
            for j in range(len(xs)):
                p = Point(); p.x = xs[j]; p.y = ys[j]
                m.points.append(p)
            marker_array.markers.append(m)
        self.traj_pub.publish(marker_array)

    # ================================================================
    # ROTATE IN PLACE
    # ================================================================

    def rotate_in_place(self, target):
        rx, ry, ryaw = self.robot_state[0], self.robot_state[1], self.robot_state[2]
        yaw_err = self.normalize_angle(math.atan2(target[1]-ry, target[0]-rx) - ryaw)
        cmd = Twist()
        if abs(yaw_err) < self.yaw_tol:
            self.get_logger().info("Aligned -> FOLLOWING")
            self.state = PlannerState.FOLLOWING
        else:
            w = self.rot_kp * yaw_err
            if abs(w) < self.min_w:
                w = self.min_w * (1 if w > 0 else -1)
            cmd.angular.z = float(max(min(w, self.max_w), -self.max_w))
        self.cmd_vel_pub.publish(cmd)

    # ================================================================
    # DWA CORE
    # ================================================================

    def run_dwa(self, target) -> bool:
        cur_v = self.robot_state[3]
        cur_w = self.robot_state[4]
        min_v = max(self.min_v,  cur_v - self.max_accel * self.dt)
        max_v = min(self.max_v,  cur_v + self.max_accel * self.dt)
        min_w = max(-self.max_w, cur_w - self.max_dyaw  * self.dt)
        max_w = min(self.max_w,  cur_w + self.max_dyaw  * self.dt)
        v_range = np.arange(min_v, max_v, self.v_res)
        w_range = np.arange(min_w, max_w, self.w_res)
        if len(v_range) == 0: v_range = np.array([cur_v])
        if len(w_range) == 0: w_range = np.array([cur_w])

        rx, ry, rtheta = self.robot_state[0], self.robot_state[1], self.robot_state[2]
        candidates = []

        for v in v_range:
            for w in w_range:
                thetas = rtheta + w * self.time_steps
                xs     = rx + np.cumsum(v * np.cos(thetas) * self.dt)
                ys     = ry + np.cumsum(v * np.sin(thetas) * self.dt)

                # Costmap: 1 lần cho cả lọc lẫn scoring
                traj_cost = self.check_trajectory_collision_batch(xs, ys)
                if traj_cost >= self.lethal_cost:
                    continue

                end_x, end_y = xs[-1], ys[-1]
                c_obs = float(self.lethal_cost - traj_cost)

                error_angle = self.normalize_angle(math.atan2(target[1]-end_y, target[0]-end_x) - thetas[-1])
                abs_err = abs(error_angle)
                c_head  = math.pi - abs_err
                c_vel   = abs(float(v))
                c_ghead = math.pi - abs_err

                c_dynobs = self.compute_dist_dynamic(xs, ys)
                if c_dynobs <= self.dynobs_lethal:
                    continue

                candidates.append((xs, ys, float(v), float(w), c_head, c_obs, c_vel, c_ghead, c_dynobs))

        cmd         = Twist()
        dwa_success = len(candidates) > 0

        if dwa_success:
            raw = np.array([[c[4], c[5], c[6], c[7], c[8]] for c in candidates])
            mins   = raw.min(axis=0)
            maxs   = raw.max(axis=0)
            ranges = maxs - mins
            safe_ranges = np.where(ranges > 1e-9, ranges, 1.0)
            norm        = np.where(ranges > 1e-9, (raw - mins) / safe_ranges, 1.0)
            weights = np.array([self.p_head, self.p_dist, self.p_vel, self.p_ghead, self.p_dynobs])
            scores   = norm @ weights
            best_idx = int(np.argmax(scores))
            cmd.linear.x  = candidates[best_idx][2]
            cmd.angular.z = candidates[best_idx][3]
            best_dyn = candidates[best_idx][8]
            if best_dyn < 0.3:
                self.get_logger().warn(f"⚠️ dist_dyn={best_dyn:.3f} — robot gần vật cản động!")
        else:
            best_idx = -1
            self.get_logger().warn("DWA: Không có trajectory hợp lệ! Dừng robot.")

        all_paths = [(c[0], c[1]) for c in candidates]
        if all_paths:
            self.visualize_trajectories(all_paths, best_idx)

        self.cmd_vel_pub.publish(cmd)
        return dwa_success

    # ================================================================
    # FOLLOWING LOGIC
    # ================================================================

    def _run_following_logic(self):
        if not self.waypoints:
            self.cmd_vel_pub.publish(Twist())
            return

        self.update_waypoint_progress()

        final_wp     = self.waypoints[-1]
        dist_to_goal = math.hypot(self.robot_state[0] - final_wp[0],
                                  self.robot_state[1] - final_wp[1])

        if dist_to_goal < self.goal_tol:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("🏁 GOAL REACHED!")
            self.clear_trajectories()  # ← XÓA VISUALIZATION KHI ĐẠT GOAL
            self.cancel_replan()
            self.state          = PlannerState.IDLE
            self.waypoints      = []
            self.smooth_path    = []
            self.current_wp_idx = 0
            self.standstill_initialized = False
            return

        # Final approach zone: tránh DWA gây quay vòng gần đích

        APPROACH_ZONE = self.goal_tol * 2.0
        if dist_to_goal < APPROACH_ZONE:
            cmd = Twist()
            rx, ry, ryaw = self.robot_state[0], self.robot_state[1], self.robot_state[2]
            yaw_to_goal   = math.atan2(final_wp[1] - ry, final_wp[0] - rx)
            yaw_err       = self.normalize_angle(yaw_to_goal - ryaw)
            cmd.linear.x  = float(min(self.max_v * 0.4, dist_to_goal * 0.8))
            cmd.angular.z = float(max(min(self.rot_kp * yaw_err, self.max_w * 0.5), -self.max_w * 0.5))
            self.cmd_vel_pub.publish(cmd)
            return

        local_goal = self.get_current_waypoint()
        if local_goal is None:
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.global_frame
        goal_msg.pose.position.x = local_goal[0]
        goal_msg.pose.position.y = local_goal[1]
        self.local_goal_pub.publish(goal_msg)

        self.run_dwa(local_goal)

        if self.state == PlannerState.FOLLOWING and self.replan_enabled:
            if self.check_standstill():
                self.trigger_replan(reason='standstill')
            elif self.check_deviation():
                self.trigger_replan(reason='path deviation')

    # ================================================================
    # MAIN CONTROL LOOP
    # ================================================================

    def control_loop(self):
        if not self.update_pose_from_tf():
            return

        if self.state == PlannerState.ROTATING:
            if not self.waypoints:
                return
            target = self.waypoints[1] if len(self.waypoints) > 1 else self.waypoints[0]
            self.rotate_in_place(target)

        elif self.state in [PlannerState.FOLLOWING, PlannerState.REPLANNING]:
            self._run_following_logic()

        elif self.state == PlannerState.IDLE:
            self.cmd_vel_pub.publish(Twist())


# ====================================================================
# ENTRY POINT
# ====================================================================

def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()