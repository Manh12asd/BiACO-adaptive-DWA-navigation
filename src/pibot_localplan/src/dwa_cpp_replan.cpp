#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "obstacle_detector/msg/obstacles.hpp"
#include "pibot_msg/action/aco_plan.hpp"

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

enum class PlannerState {
    IDLE = 0,
    ROTATING = 1,
    FOLLOWING = 2,
    REPLANNING = 3
};

struct DynamicCircle {
    double cx, cy, radius, vx, vy;
};

struct TrajectoryCandidate {
    std::vector<double> xs;
    std::vector<double> ys;
    double v, w;
    double c_head, c_obs, c_vel, c_dynobs;
};

class DWAPlanner : public rclcpp::Node {
public:
    using AcoPlan = pibot_msg::action::AcoPlan;
    using GoalHandleAcoPlan = rclcpp_action::ClientGoalHandle<AcoPlan>;

    DWAPlanner() : Node("dwa_planner_node"), state_(PlannerState::IDLE), current_wp_idx_(0), 
                   is_replanning_(false), active_goal_handle_(nullptr), 
                   last_periodic_replan_time_(0.0), standstill_initialized_(false) {

        // --- CẤU HÌNH THAM SỐ ---
        max_v_ = declare_parameter("max_speed", 0.12);
        min_v_ = declare_parameter("min_speed", 0.0);
        max_w_ = declare_parameter("max_yaw_rate", 3.0);
        min_w_ = declare_parameter("min_yaw_rate", 0.2);
        max_accel_ = declare_parameter("max_accel", 0.5);
        max_dyaw_ = declare_parameter("max_dyaw_rate", 6.4);

        // Giới hạn động cơ
        wheel_base_ = declare_parameter("wheel_base", 0.17);
        max_wheel_speed_ = declare_parameter("max_wheel_speed", 0.34);

        v_res_ = declare_parameter("v_res", 0.01);
        w_res_ = declare_parameter("w_res", 0.1);
        dt_ = declare_parameter("dt", 0.1);
        predict_time_ = declare_parameter("predict_time", 3.0);
        goal_tol_ = declare_parameter("goal_tolerance", 0.2);
        yaw_tol_ = declare_parameter("yaw_goal_tolerance", 0.4);
        rot_kp_ = declare_parameter("rotation_kp", 1.5);
        lethal_cost_ = declare_parameter("lethal_cost", 90);

        p_head_ = declare_parameter("alpha_heading", 0.20);
        p_dist_ = declare_parameter("beta_dist", 0.45);
        p_vel_ = declare_parameter("gamma_vel", 0.25);

        global_frame_ = declare_parameter("global_frame", "map");
        robot_frame_ = declare_parameter("robot_frame", "base_footprint");

        // --- REPLAN PARAMETERS ---
        replan_enabled_ = declare_parameter("replan_enabled", true);
        replan_mode_ = declare_parameter("replan_mode", "event_driven"); // "event_driven" hoặc "periodic"
        replan_period_ = declare_parameter("replan_period", 1.0);        // Chu kỳ tính bằng giây
        replan_deviation_dist_ = declare_parameter("replan_deviation_dist", 0.5);
        replan_cooldown_ = declare_parameter("replan_cooldown", 0.3);
        
        stuck_time_threshold_ = declare_parameter("stuck_time_threshold", 3.0);
        stuck_dist_threshold_ = declare_parameter("stuck_dist_threshold", 0.05);

        R_robot_ = declare_parameter("robot_radius", 0.20);
        c_safe_ = declare_parameter("c_safe", 0.1);
        k_dynamic_ = declare_parameter("k_dynamic", 3.0);
        p_dynobs_ = declare_parameter("epsilon_dynobs", 0.0);
        dynobs_lethal_ = declare_parameter("dynobs_lethal", 0.0);
        dynobs_range_ = declare_parameter("dynobs_range", 2.5);

        num_steps_ = static_cast<int>(predict_time_ / dt_);

        // Khởi tạo trạng thái [x, y, theta, v, w]
        robot_state_ = {0.0, 0.0, 0.0, 0.0, 0.0};

        // --- QoS Profiles ---
        rclcpp::QoS qos_sensor = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        rclcpp::QoS qos_map = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        // --- Publishers & Subscriptions ---
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/pibot_controller/cmd_vel_unstamped", 10);
        local_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/local_goal", 10);
        traj_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visual_paths", 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", qos_sensor, std::bind(&DWAPlanner::velocity_callback, this, _1));
        costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_costmap/costmap", qos_map, std::bind(&DWAPlanner::costmap_callback, this, _1));
        wp_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/improve_aco3/waypoints", 10, std::bind(&DWAPlanner::waypoints_callback, this, _1));
        smooth_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/improve_aco3/path", 10, std::bind(&DWAPlanner::smooth_path_callback, this, _1));
        dynobs_sub_ = create_subscription<obstacle_detector::msg::Obstacles>(
            "/dynamic_obstacles", 10, std::bind(&DWAPlanner::dynamic_obs_callback, this, _1));

        // --- TF ---
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // --- Action Client ---
        action_client_ = rclcpp_action::create_client<AcoPlan>(this, "aco_plan");

        // --- Timer ---
        timer_ = create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&DWAPlanner::control_loop, this));

        RCLCPP_INFO(get_logger(), "DWA Planner Started | L=%.2fm, V_wheel_max=%.2fm/s", wheel_base_, max_wheel_speed_);
        RCLCPP_INFO(get_logger(), "Replan Mode: %s (Enabled: %d)", replan_mode_.c_str(), replan_enabled_);
    }

private:
    // Parameters
    double max_v_, min_v_, max_w_, min_w_, max_accel_, max_dyaw_;
    double wheel_base_, max_wheel_speed_;
    double v_res_, w_res_, dt_, predict_time_, goal_tol_, yaw_tol_, rot_kp_;
    int lethal_cost_;
    double p_head_, p_dist_, p_vel_, p_dynobs_;
    std::string global_frame_, robot_frame_;
    bool replan_enabled_;
    std::string replan_mode_;
    double replan_period_;
    double replan_deviation_dist_, replan_cooldown_, stuck_time_threshold_, stuck_dist_threshold_;
    double R_robot_, c_safe_, k_dynamic_, dynobs_lethal_, dynobs_range_;

    int num_steps_;
    std::vector<double> robot_state_; // 0:x, 1:y, 2:theta, 3:v, 4:w
    std::vector<std::pair<double, double>> waypoints_;
    std::vector<std::pair<double, double>> smooth_path_;
    geometry_msgs::msg::PoseStamped final_goal_pose_;

    PlannerState state_;
    size_t current_wp_idx_;
    std::vector<int8_t> costmap_data_;
    nav_msgs::msg::MapMetaData costmap_info_;
    bool has_costmap_ = false;

    // --- Biến quản lý Replan an toàn ---
    bool is_replanning_;
    std::shared_ptr<GoalHandleAcoPlan> active_goal_handle_;
    
    double last_replan_time_;
    double last_periodic_replan_time_;
    double last_moved_time_;
    std::pair<double, double> last_check_pos_;
    bool standstill_initialized_;
    std::vector<DynamicCircle> dynamic_circles_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr wp_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr smooth_sub_;
    rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr dynobs_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp_action::Client<AcoPlan>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ================================================================
    // ODOMETRY / TF
    // ================================================================

    void velocity_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_state_[3] = msg->twist.twist.linear.x;
        robot_state_[4] = msg->twist.twist.angular.z;
    }

    bool update_pose_from_tf() {
        try {
            auto transform = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
            robot_state_[0] = transform.transform.translation.x;
            robot_state_[1] = transform.transform.translation.y;
            tf2::Quaternion q(
                transform.transform.rotation.x, transform.transform.rotation.y,
                transform.transform.rotation.z, transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            robot_state_[2] = yaw;
            return true;
        } catch (const tf2::TransformException &ex) {
            return false;
        }
    }

    // ================================================================
    // COSTMAP & PATH CALLBACKS
    // ================================================================

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (!has_costmap_) {
            RCLCPP_INFO(get_logger(), "Map Received: %dx%d", msg->info.width, msg->info.height);
            has_costmap_ = true;
        }
        costmap_info_ = msg->info;
        costmap_data_ = msg->data;
    }

    void waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        waypoints_.clear();
        for (const auto& p : msg->poses) {
            waypoints_.emplace_back(p.pose.position.x, p.pose.position.y);
        }
        if (waypoints_.empty()) return;

        final_goal_pose_ = msg->poses.back();
        current_wp_idx_ = 0;
        
        // Cập nhật mốc thời gian để không bị kích hoạt periodic replan ngay lập tức
        last_periodic_replan_time_ = now().seconds();
        reset_standstill();

        if (state_ == PlannerState::IDLE || state_ == PlannerState::FOLLOWING || state_ == PlannerState::REPLANNING) {
            auto next_wp = waypoints_.size() > 1 ? waypoints_[1] : waypoints_[0];
            double yaw_err = compute_yaw_error_to(next_wp.first, next_wp.second);
            if (yaw_err < yaw_tol_ * 3.0) {
                state_ = PlannerState::FOLLOWING;
                RCLCPP_INFO(get_logger(), "Path received, yaw_err=%.1f deg -> FOLLOWING", yaw_err * 180.0 / M_PI);
            } else {
                state_ = PlannerState::ROTATING;
                RCLCPP_INFO(get_logger(), "Path received, yaw_err=%.1f deg -> ROTATING", yaw_err * 180.0 / M_PI);
            }
        }
    }

    void smooth_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        smooth_path_.clear();
        for (const auto& p : msg->poses) {
            smooth_path_.emplace_back(p.pose.position.x, p.pose.position.y);
        }
    }

    void dynamic_obs_callback(const obstacle_detector::msg::Obstacles::SharedPtr msg) {
        dynamic_circles_.clear();
        for (const auto& c : msg->circles) {
            dynamic_circles_.push_back({c.center.x, c.center.y, c.true_radius, c.velocity.x, c.velocity.y});
        }
    }

    // ================================================================
    // DIST_DYNAMIC
    // ================================================================

    double compute_dist_dynamic(const std::vector<double>& xs, const std::vector<double>& ys) {
        if (dynamic_circles_.empty()) return 1.0;

        double worst = 1.0;
        for (const auto& obs : dynamic_circles_) {
            double min_d = std::numeric_limits<double>::max();
            double obs_worst_step = 1.0;

            for (size_t i = 0; i < xs.size(); ++i) {
                double t = (i + 1) * dt_;
                double x_obs = obs.cx + obs.vx * t;
                double y_obs = obs.cy + obs.vy * t;
                double dx = xs[i] - x_obs;
                double dy = ys[i] - y_obs;
                double d = std::hypot(dx, dy);

                min_d = std::min(min_d, d);

                double delta_d = d - (R_robot_ + obs.radius + c_safe_);
                double dist_step = std::clamp(1.0 - std::exp(-k_dynamic_ * delta_d), 0.0, 1.0);
                obs_worst_step = std::min(obs_worst_step, dist_step);
            }

            if (min_d <= dynobs_range_ && obs_worst_step < worst) {
                worst = obs_worst_step;
            }
        }
        return worst;
    }

    // ================================================================
    // STANDSTILL DETECTION & REPLANNING
    // ================================================================

    void reset_standstill() {
        last_moved_time_ = now().seconds();
        last_check_pos_ = {robot_state_[0], robot_state_[1]};
        standstill_initialized_ = true;
    }

    bool check_standstill() {
        if (!standstill_initialized_) {
            reset_standstill();
            return false;
        }
        double current_time = now().seconds();
        double dist_moved = std::hypot(robot_state_[0] - last_check_pos_.first, robot_state_[1] - last_check_pos_.second);

        if (dist_moved >= stuck_dist_threshold_) {
            last_moved_time_ = current_time;
            last_check_pos_ = {robot_state_[0], robot_state_[1]};
            return false;
        }

        double standstill_duration = current_time - last_moved_time_;
        if (standstill_duration >= stuck_time_threshold_) {
            RCLCPP_WARN(get_logger(), "Standstill: Robot hasn't moved for %.1fs (dist=%.3fm)", standstill_duration, dist_moved);
            return true;
        }
        return false;
    }

    bool check_deviation() {
        if (smooth_path_.empty()) return false;

        double min_d = std::numeric_limits<double>::max();
        for (const auto& p : smooth_path_) {
            double d = std::hypot(p.first - robot_state_[0], p.second - robot_state_[1]);
            min_d = std::min(min_d, d);
        }
        if (min_d > replan_deviation_dist_) {
            RCLCPP_WARN(get_logger(), "Deviation: %.2fm > %.2fm", min_d, replan_deviation_dist_);
            return true;
        }
        return false;
    }

    void cancel_replan() {
        if (active_goal_handle_) {
            action_client_->async_cancel_goal(active_goal_handle_);
            active_goal_handle_.reset();
        }
        is_replanning_ = false;
    }

    void trigger_replan(const std::string& reason = "") {
        // Kiểm tra an toàn: Không kích hoạt thêm nếu đang replan rồi
        if (!replan_enabled_ || is_replanning_ || waypoints_.empty()) return;

        double current_time = now().seconds();
        if ((current_time - last_replan_time_) < replan_cooldown_) return;

        // Giảm wait_for xuống 10ms để không block node lâu
        if (!action_client_->wait_for_action_server(std::chrono::milliseconds(10))) {
            RCLCPP_WARN(get_logger(), "ACO action server not available, skip replan.");
            return;
        }

        is_replanning_ = true;
        last_replan_time_ = current_time;
        reset_standstill();
        state_ = PlannerState::REPLANNING;
        RCLCPP_INFO(get_logger(), "Triggering replan [%s]...", reason.c_str());

        auto goal_msg = AcoPlan::Goal();
        goal_msg.goal_pose = final_goal_pose_;

        auto send_goal_options = rclcpp_action::Client<AcoPlan>::SendGoalOptions();
        
        // Lưu trữ Goal Handle để có thể quản lý việc Cancel sau này
        send_goal_options.goal_response_callback = [this](const GoalHandleAcoPlan::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_WARN(get_logger(), "Replan goal REJECTED.");
                is_replanning_ = false;
                state_ = PlannerState::FOLLOWING;
            } else {
                active_goal_handle_ = goal_handle;
            }
        };
        
        send_goal_options.feedback_callback = [this](GoalHandleAcoPlan::SharedPtr, const std::shared_ptr<const AcoPlan::Feedback> feedback) {
            RCLCPP_DEBUG(get_logger(), "Replan: iter=%d, best=%.2f", feedback->iteration, feedback->current_best_length);
        };
        
        // Callback khi có Result (thành công hoặc thất bại/bị hủy)
        send_goal_options.result_callback = [this](const GoalHandleAcoPlan::WrappedResult & result) {
            is_replanning_ = false;       // Mở khóa để cho phép replan lần tới
            active_goal_handle_.reset();  // Dọn dẹp con trỏ
            
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->path.poses.size() > 0) {
                RCLCPP_INFO(get_logger(), "Replan success! length=%.2f", result.result->total_length);
            } else {
                RCLCPP_WARN(get_logger(), "Replan failed, canceled or empty path.");
                state_ = PlannerState::FOLLOWING;
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // ================================================================
    // HELPERS & COSTMAP CHECK
    // ================================================================

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double compute_yaw_error_to(double tx, double ty) {
        return std::abs(normalize_angle(std::atan2(ty - robot_state_[1], tx - robot_state_[0]) - robot_state_[2]));
    }

    int check_trajectory_collision_batch(const std::vector<double>& xs, const std::vector<double>& ys) {
        if (!has_costmap_) return 100;

        double res = costmap_info_.resolution;
        double ox = costmap_info_.origin.position.x;
        double oy = costmap_info_.origin.position.y;
        int w = costmap_info_.width;
        int h = costmap_info_.height;
        int max_cost = 0;

        for (size_t i = 0; i < xs.size(); ++i) {
            int mx = static_cast<int>((xs[i] - ox) / res);
            int my = static_cast<int>((ys[i] - oy) / res);

            if (mx < 0 || mx >= w || my < 0 || my >= h) return 100;

            int8_t cost = costmap_data_[my * w + mx];
            int final_cost = (cost == -1) ? 100 : cost;
            max_cost = std::max(max_cost, final_cost);
            if (max_cost >= lethal_cost_) break;
        }
        return max_cost;
    }

    void update_waypoint_progress() {
        if (waypoints_.empty()) return;

        while (current_wp_idx_ < waypoints_.size() - 1) {
            auto wp = waypoints_[current_wp_idx_];
            auto next_wp = waypoints_[current_wp_idx_ + 1];

            double dist = std::hypot(robot_state_[0] - wp.first, robot_state_[1] - wp.second);
            double wp_tol = goal_tol_ * 2.0;

            double seg_x = next_wp.first - wp.first;
            double seg_y = next_wp.second - wp.second;
            double rob_x = robot_state_[0] - wp.first;
            double rob_y = robot_state_[1] - wp.second;

            double seg_len_sq = seg_x * seg_x + seg_y * seg_y;
            double dot = seg_x * rob_x + seg_y * rob_y;
            double t = (seg_len_sq > 0.0) ? (dot / seg_len_sq) : 0.0;

            if (dist < wp_tol || t > 0.0) {
                RCLCPP_INFO(get_logger(), "WP %zu -> %zu/%zu (dist=%.2fm, t=%.2f)",
                            current_wp_idx_, current_wp_idx_ + 1, waypoints_.size() - 1, dist, t);
                current_wp_idx_++;
            } else {
                break;
            }
        }
    }

    // ================================================================
    // VISUALIZATION
    // ================================================================

    void clear_trajectories() {
        visualization_msgs::msg::MarkerArray msg;
        visualization_msgs::msg::Marker del_m;
        del_m.action = visualization_msgs::msg::Marker::DELETEALL;
        msg.markers.push_back(del_m);
        traj_pub_->publish(msg);
    }

    void visualize_trajectories(const std::vector<TrajectoryCandidate>& candidates, int best_idx) {
        visualization_msgs::msg::MarkerArray msg;
        visualization_msgs::msg::Marker del_m;
        del_m.action = visualization_msgs::msg::Marker::DELETEALL;
        msg.markers.push_back(del_m);

        int step = 3;
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (static_cast<int>(i) != best_idx && i % step != 0) continue;

            visualization_msgs::msg::Marker m;
            m.header.frame_id = global_frame_;
            m.header.stamp = this->now();
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.02;

            if (static_cast<int>(i) == best_idx) {
                m.ns = "dwa_best"; m.id = 9999;
                m.color.r = 1.0; m.color.a = 1.0;
            } else {
                m.ns = "dwa_paths"; m.id = i;
                m.color.r = 1.0; m.color.a = 1.0;
            }

            for (size_t j = 0; j < candidates[i].xs.size(); ++j) {
                geometry_msgs::msg::Point p;
                p.x = candidates[i].xs[j];
                p.y = candidates[i].ys[j];
                m.points.push_back(p);
            }
            msg.markers.push_back(m);
        }
        traj_pub_->publish(msg);
    }

    // ================================================================
    // DWA CORE & CONTROL LOGIC
    // ================================================================

    void rotate_in_place(double tx, double ty) {
        double yaw_err = normalize_angle(std::atan2(ty - robot_state_[1], tx - robot_state_[0]) - robot_state_[2]);
        geometry_msgs::msg::Twist cmd;

        if (std::abs(yaw_err) < yaw_tol_) {
            RCLCPP_INFO(get_logger(), "Aligned -> FOLLOWING");
            state_ = PlannerState::FOLLOWING;
        } else {
            double w = rot_kp_ * yaw_err;
            if (std::abs(w) < min_w_) w = min_w_ * (w > 0 ? 1 : -1);
            cmd.angular.z = std::clamp(w, -max_w_, max_w_);
        }
        cmd_vel_pub_->publish(cmd);
    }

    bool run_dwa(double tx, double ty) {
        double cur_v = robot_state_[3];
        double cur_w = robot_state_[4];

        double dyn_min_v = std::max(min_v_, cur_v - max_accel_ * dt_);
        double dyn_max_v = std::min(max_v_, cur_v + max_accel_ * dt_);
        double dyn_min_w = std::max(-max_w_, cur_w - max_dyaw_ * dt_);
        double dyn_max_w = std::min(max_w_, cur_w + max_dyaw_ * dt_);

        std::vector<TrajectoryCandidate> candidates;

        for (double v = dyn_min_v; v <= dyn_max_v; v += v_res_) {
            for (double w = dyn_min_w; w <= dyn_max_w; w += w_res_) {

                // Ràng buộc động cơ
                double v_R = v + (w * wheel_base_ / 2.0);
                double v_L = v - (w * wheel_base_ / 2.0);
                if (std::abs(v_R) > max_wheel_speed_ || std::abs(v_L) > max_wheel_speed_) continue;

                std::vector<double> xs, ys, thetas;
                double cur_x = robot_state_[0];
                double cur_y = robot_state_[1];
                double cur_theta = robot_state_[2];

                for (int i = 1; i <= num_steps_; ++i) {
                    cur_theta += w * dt_;
                    cur_x += v * std::cos(cur_theta) * dt_;
                    cur_y += v * std::sin(cur_theta) * dt_;
                    xs.push_back(cur_x);
                    ys.push_back(cur_y);
                    thetas.push_back(cur_theta);
                }

                int traj_cost = check_trajectory_collision_batch(xs, ys);
                if (traj_cost >= lethal_cost_) continue;

                double error_angle = normalize_angle(std::atan2(ty - ys.back(), tx - xs.back()) - thetas.back());
                double c_head   = M_PI - std::abs(error_angle);          // ∈ [0, π]
                double c_obs    = static_cast<double>(lethal_cost_ - traj_cost); // ∈ [0, lethal_cost_]
                double c_vel    = std::abs(v);                            // ∈ [0, max_v_]
                double c_dynobs = compute_dist_dynamic(xs, ys);           // ∈ [0, 1]

                if (c_dynobs <= dynobs_lethal_) continue;

                candidates.push_back({xs, ys, v, w, c_head, c_obs, c_vel, c_dynobs});
            }
        }

        geometry_msgs::msg::Twist cmd;
        if (candidates.empty()) {
            RCLCPP_WARN(get_logger(), "DWA: Không có trajectory hợp lệ! Dừng robot.");
            cmd_vel_pub_->publish(cmd);
            return false;
        }

        // --- Chuẩn hóa theo miền giá trị lý thuyết cố định ---
        const double norm_head   = M_PI;
        const double norm_obs    = static_cast<double>(lethal_cost_);
        const double norm_vel    = (max_v_ > 1e-9) ? max_v_ : 1.0;
        const double norm_dynobs = 1.0;

        int best_idx = -1;
        double best_score = std::numeric_limits<double>::lowest();

        for (size_t i = 0; i < candidates.size(); ++i) {
            double n_head   = candidates[i].c_head   / norm_head;
            double n_obs    = candidates[i].c_obs    / norm_obs;
            double n_vel    = candidates[i].c_vel    / norm_vel;
            double n_dynobs = candidates[i].c_dynobs / norm_dynobs;

            double score = p_head_   * n_head
                         + p_dist_   * n_obs
                         + p_vel_    * n_vel
                         + p_dynobs_ * n_dynobs;

            if (score > best_score) {
                best_score = score;
                best_idx = static_cast<int>(i);
            }
        }

        cmd.linear.x  = candidates[best_idx].v;
        cmd.angular.z = candidates[best_idx].w;

        if (candidates[best_idx].c_dynobs < 0.3) {
            RCLCPP_WARN(get_logger(), "⚠️ dist_dyn=%.3f — robot gần vật cản động!", candidates[best_idx].c_dynobs);
        }

        visualize_trajectories(candidates, best_idx);
        cmd_vel_pub_->publish(cmd);
        return true;
    }

    void run_following_logic() {
        if (waypoints_.empty()) {
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
            return;
        }

        update_waypoint_progress();

        auto final_wp = waypoints_.back();
        double dist_to_goal = std::hypot(robot_state_[0] - final_wp.first, robot_state_[1] - final_wp.second);

        if (dist_to_goal < goal_tol_) {
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
            RCLCPP_INFO(get_logger(), "🏁 GOAL REACHED!");
            clear_trajectories();

            // Nếu đã tới đích, Hủy mọi tiến trình replan đang chạy dang dở
            cancel_replan(); 

            state_ = PlannerState::IDLE;
            waypoints_.clear();
            smooth_path_.clear();
            current_wp_idx_ = 0;
            standstill_initialized_ = false;
            return;
        }

        if (current_wp_idx_ >= waypoints_.size()) return;
        auto local_goal = waypoints_[current_wp_idx_];

        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.frame_id = global_frame_;
        goal_msg.header.stamp = now();
        goal_msg.pose.position.x = local_goal.first;
        goal_msg.pose.position.y = local_goal.second;
        local_goal_pub_->publish(goal_msg);

        run_dwa(local_goal.first, local_goal.second);

        // --- KIỂM TRA ĐIỀU KIỆN REPLAN ---
        if (state_ == PlannerState::FOLLOWING && replan_enabled_) {
            double current_time = now().seconds();
            
            if (replan_mode_ == "periodic") {
                // Chế độ 1: Replan theo chu kỳ thời gian
                if ((current_time - last_periodic_replan_time_) >= replan_period_) {
                    trigger_replan("periodic");
                    // Update timer chỉ khi replan thực sự được kích hoạt thành công (hoặc đợi trigger tự xử lý timeout cooldown)
                    if (is_replanning_) {
                        last_periodic_replan_time_ = current_time;
                    }
                }
            } else {
                // Chế độ 2: Replan như cũ (Event-driven)
                if (check_standstill()) {
                    trigger_replan("standstill");
                } else if (check_deviation()) {
                    trigger_replan("path deviation");
                }
            }
        }
    }

    void control_loop() {
        if (!update_pose_from_tf()) return;

        if (state_ == PlannerState::ROTATING) {
            if (waypoints_.empty()) return;
            auto target = waypoints_.size() > 1 ? waypoints_[1] : waypoints_[0];
            rotate_in_place(target.first, target.second);
        } else if (state_ == PlannerState::FOLLOWING || state_ == PlannerState::REPLANNING) {
            run_following_logic();
        } else if (state_ == PlannerState::IDLE) {
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DWAPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}