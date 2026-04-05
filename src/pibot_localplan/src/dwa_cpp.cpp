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

// Custom messages and actions
#include "obstacle_detector/msg/obstacles.hpp"
#include "pibot_msg/action/aco_plan.hpp"

#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <limits>


enum class PlannerState {
    IDLE = 0,
    ROTATING = 1,
    FOLLOWING = 2,
    REPLANNING = 3
};

struct DynamicObstacle {
    double cx, cy, radius, vx, vy;
};

struct Trajectory {
    std::vector<double> xs;
    std::vector<double> ys;
    double v, w;
    double c_head, c_obs, c_vel, c_ghead, c_dynobs;
};

class DWAPlanner : public rclcpp::Node {
public:
    using AcoPlan = pibot_msg::action::AcoPlan;
    using GoalHandleAcoPlan = rclcpp_action::ClientGoalHandle<AcoPlan>;

    DWAPlanner() : Node("dwa_planner_node"), state_(PlannerState::IDLE), current_wp_idx_(0),
                   is_replanning_(false), standstill_initialized_(false) {

        // --- Declare Parameters ---
        this->declare_parameter("max_speed", 0.15);
        this->declare_parameter("min_speed", 0.0);
        this->declare_parameter("max_yaw_rate", 0.6);
        this->declare_parameter("min_yaw_rate", 0.2);
        this->declare_parameter("max_accel", 1.0);
        this->declare_parameter("max_dyaw_rate", 1.0);
        this->declare_parameter("v_res", 0.02);
        this->declare_parameter("w_res", 0.1);
        this->declare_parameter("dt", 0.1);
        this->declare_parameter("predict_time", 3.5);
        this->declare_parameter("goal_tolerance", 0.2);
        this->declare_parameter("yaw_goal_tolerance", 0.4);
        this->declare_parameter("rotation_kp", 1.5);
        this->declare_parameter("lethal_cost", 90);
        this->declare_parameter("alpha_heading", 0.3);
        this->declare_parameter("beta_dist", 1.0);
        this->declare_parameter("gamma_vel", 0.5);
        this->declare_parameter("delta_ghead", 0.0);
        this->declare_parameter("global_frame", "map");
        this->declare_parameter("robot_frame", "base_footprint");
        this->declare_parameter("replan_enabled", true);
        this->declare_parameter("replan_deviation_dist", 0.5);
        this->declare_parameter("replan_cooldown", 5.0);
        this->declare_parameter("stuck_time_threshold", 3.0);
        this->declare_parameter("stuck_dist_threshold", 0.05);
        this->declare_parameter("robot_radius", 0.20);
        this->declare_parameter("c_safe", 0.1);
        this->declare_parameter("k_dynamic", 3.0);
        this->declare_parameter("epsilon_dynobs", 0.4);
        this->declare_parameter("dynobs_lethal", 0.0);
        this->declare_parameter("dynobs_range", 2.5);

        // --- Fetch Parameters ---
        max_v_ = this->get_parameter("max_speed").as_double();
        min_v_ = this->get_parameter("min_speed").as_double();
        max_w_ = this->get_parameter("max_yaw_rate").as_double();
        min_w_ = this->get_parameter("min_yaw_rate").as_double();
        max_accel_ = this->get_parameter("max_accel").as_double();
        max_dyaw_ = this->get_parameter("max_dyaw_rate").as_double();
        v_res_ = this->get_parameter("v_res").as_double();
        w_res_ = this->get_parameter("w_res").as_double();
        dt_ = this->get_parameter("dt").as_double();
        predict_time_ = this->get_parameter("predict_time").as_double();
        goal_tol_ = this->get_parameter("goal_tolerance").as_double();
        yaw_tol_ = this->get_parameter("yaw_goal_tolerance").as_double();
        rot_kp_ = this->get_parameter("rotation_kp").as_double();
        lethal_cost_ = this->get_parameter("lethal_cost").as_int();
        p_head_ = this->get_parameter("alpha_heading").as_double();
        p_dist_ = this->get_parameter("beta_dist").as_double();
        p_vel_ = this->get_parameter("gamma_vel").as_double();
        p_ghead_ = this->get_parameter("delta_ghead").as_double();
        global_frame_ = this->get_parameter("global_frame").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        replan_enabled_ = this->get_parameter("replan_enabled").as_bool();
        replan_deviation_dist_ = this->get_parameter("replan_deviation_dist").as_double();
        replan_cooldown_ = this->get_parameter("replan_cooldown").as_double();
        stuck_time_threshold_ = this->get_parameter("stuck_time_threshold").as_double();
        stuck_dist_threshold_ = this->get_parameter("stuck_dist_threshold").as_double();
        R_robot_ = this->get_parameter("robot_radius").as_double();
        c_safe_ = this->get_parameter("c_safe").as_double();
        k_dynamic_ = this->get_parameter("k_dynamic").as_double();
        p_dynobs_ = this->get_parameter("epsilon_dynobs").as_double();
        dynobs_lethal_ = this->get_parameter("dynobs_lethal").as_double();
        dynobs_range_ = this->get_parameter("dynobs_range").as_double();

        num_steps_ = static_cast<int>(predict_time_ / dt_);
        for (int i = 1; i <= num_steps_; ++i) {
            time_steps_.push_back(i * dt_);
        }

        robot_state_ = {0.0, 0.0, 0.0, 0.0, 0.0}; // x, y, theta, v, w
        last_check_pos_ = {0.0, 0.0};

        // --- Setup ROS 2 Interfaces ---
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        action_client_ = rclcpp_action::create_client<AcoPlan>(this, "aco_plan");

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/pibot_controller/cmd_vel_unstamped", 10);
        local_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/local_goal", 10);
        traj_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visual_paths", 10);

        auto qos_sensor = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        auto qos_map = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", qos_sensor, std::bind(&DWAPlanner::velocity_callback, this, std::placeholders::_1));
        
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_costmap/costmap", qos_map, std::bind(&DWAPlanner::costmap_callback, this, std::placeholders::_1));
            
        wp_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/improve_aco3/waypoints", 10, std::bind(&DWAPlanner::waypoints_callback, this, std::placeholders::_1));
            
        smooth_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/improve_aco3/path", 10, std::bind(&DWAPlanner::smooth_path_callback, this, std::placeholders::_1));
            
        dynobs_sub_ = this->create_subscription<obstacle_detector::msg::Obstacles>(
            "/dynamic_obstacles", 10, std::bind(&DWAPlanner::dynamic_obs_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&DWAPlanner::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "DWA Planner Started | R_robot=%.2fm, c_safe=%.2fm, k_dyn=%.2f, ε_dyn=%.2f", 
                    R_robot_, c_safe_, k_dynamic_, p_dynobs_);
    }

private:
    // --- ROS 2 Parameters ---
    double max_v_, min_v_, max_w_, min_w_, max_accel_, max_dyaw_, v_res_, w_res_, dt_, predict_time_;
    double goal_tol_, yaw_tol_, rot_kp_, p_head_, p_dist_, p_vel_, p_ghead_;
    double replan_deviation_dist_, replan_cooldown_, stuck_time_threshold_, stuck_dist_threshold_;
    double R_robot_, c_safe_, k_dynamic_, p_dynobs_, dynobs_lethal_, dynobs_range_;
    int lethal_cost_;
    bool replan_enabled_;
    std::string global_frame_, robot_frame_;

    // --- State Variables ---
    PlannerState state_;
    std::vector<double> robot_state_; // [x, y, theta, v, w]
    std::vector<std::pair<double, double>> waypoints_;
    std::vector<std::pair<double, double>> smooth_path_;
    geometry_msgs::msg::PoseStamped final_goal_pose_;
    int current_wp_idx_;
    std::vector<int8_t> costmap_data_;
    nav_msgs::msg::MapMetaData costmap_info_;
    bool map_received_ = false;

    int num_steps_;
    std::vector<double> time_steps_;
    std::vector<DynamicObstacle> dynamic_circles_;

    bool is_replanning_;
    double last_replan_time_;
    double last_moved_time_;
    std::pair<double, double> last_check_pos_;
    bool standstill_initialized_;

    // --- ROS 2 Components ---
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp_action::Client<AcoPlan>::SharedPtr action_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr wp_sub_, smooth_sub_;
    rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr dynobs_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ================================================================
    // CALLBACKS
    // ================================================================

    void velocity_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_state_[3] = msg->twist.twist.linear.x;
        robot_state_[4] = msg->twist.twist.angular.z;
    }

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (!map_received_) {
            RCLCPP_INFO(this->get_logger(), "Map Received: %dx%d", msg->info.width, msg->info.height);
            map_received_ = true;
        }
        costmap_info_ = msg->info;
        costmap_data_ = msg->data;
    }

    void waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        waypoints_.clear();
        for (const auto& p : msg->poses) {
            waypoints_.push_back({p.pose.position.x, p.pose.position.y});
        }
        if (waypoints_.empty()) return;

        final_goal_pose_ = msg->poses.back();
        current_wp_idx_ = 0;
        is_replanning_ = false;
        reset_standstill();

        if (state_ == PlannerState::IDLE || state_ == PlannerState::FOLLOWING || state_ == PlannerState::REPLANNING) {
            auto next_wp = (waypoints_.size() > 1) ? waypoints_[1] : waypoints_[0];
            double yaw_err = compute_yaw_error_to(next_wp.first, next_wp.second);
            if (yaw_err < yaw_tol_ * 3.0) {
                state_ = PlannerState::FOLLOWING;
                RCLCPP_INFO(this->get_logger(), "Path received, yaw_err=%.1f deg -> FOLLOWING", yaw_err * 180.0 / M_PI);
            } else {
                state_ = PlannerState::ROTATING;
                RCLCPP_INFO(this->get_logger(), "Path received, yaw_err=%.1f deg -> ROTATING", yaw_err * 180.0 / M_PI);
            }
        }
    }

    void smooth_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        smooth_path_.clear();
        for (const auto& p : msg->poses) {
            smooth_path_.push_back({p.pose.position.x, p.pose.position.y});
        }
    }

    void dynamic_obs_callback(const obstacle_detector::msg::Obstacles::SharedPtr msg) {
        dynamic_circles_.clear();
        for (const auto& c : msg->circles) {
            dynamic_circles_.push_back({c.center.x, c.center.y, c.true_radius, c.velocity.x, c.velocity.y});
        }
    }

    // ================================================================
    // HELPERS
    // ================================================================

    bool update_pose_from_tf() {
        try {
            auto t = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
            robot_state_[0] = t.transform.translation.x;
            robot_state_[1] = t.transform.translation.y;
            robot_state_[2] = tf2::getYaw(t.transform.rotation);
            return true;
        } catch (const tf2::TransformException & ex) {
            return false;
        }
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double compute_yaw_error_to(double tx, double ty) {
        double rx = robot_state_[0], ry = robot_state_[1], ryaw = robot_state_[2];
        return std::abs(normalize_angle(std::atan2(ty - ry, tx - rx) - ryaw));
    }

    // ================================================================
    // DYNAMIC & STATIC COLLISION
    // ================================================================

    double compute_dist_dynamic(const std::vector<double>& xs, const std::vector<double>& ys) {
        if (dynamic_circles_.empty()) return 1.0;

        double worst = 1.0;
        for (const auto& obs : dynamic_circles_) {
            double obs_worst = 1.0;
            for (size_t i = 0; i < xs.size(); ++i) {
                double t = time_steps_[i];
                double x_obs = obs.cx + obs.vx * t;
                double y_obs = obs.cy + obs.vy * t;
                double dx = xs[i] - x_obs;
                double dy = ys[i] - y_obs;
                double d = std::hypot(dx, dy);

                if (d > dynobs_range_) continue;

                double delta_d = d - (R_robot_ + obs.radius + c_safe_);
                double dist_step = std::clamp(1.0 - std::exp(-k_dynamic_ * delta_d), 0.0, 1.0);
                if (dist_step < obs_worst) {
                    obs_worst = dist_step;
                }
            }
            if (obs_worst < worst) {
                worst = obs_worst;
            }
        }
        return worst;
    }

    int check_trajectory_collision(const std::vector<double>& xs, const std::vector<double>& ys) {
        if (!map_received_) return 100;
        double res = costmap_info_.resolution;
        double ox = costmap_info_.origin.position.x;
        double oy = costmap_info_.origin.position.y;
        int w = costmap_info_.width;
        int h = costmap_info_.height;

        int max_cost = 0;
        for (size_t i = 0; i < xs.size(); ++i) {
            int mx = static_cast<int>((xs[i] - ox) / res);
            int my = static_cast<int>((ys[i] - oy) / res);

            if (mx < 0 || my < 0 || mx >= w || my >= h) return 100;

            int8_t cost = costmap_data_[my * w + mx];
            int c = (cost == -1) ? 100 : static_cast<int>(cost);
            if (c > max_cost) max_cost = c;
            if (max_cost >= lethal_cost_) return max_cost;
        }
        return max_cost;
    }

    // ================================================================
    // STANDSTILL & REPLAN
    // ================================================================

    void reset_standstill() {
        last_moved_time_ = this->now().seconds();
        last_check_pos_ = {robot_state_[0], robot_state_[1]};
        standstill_initialized_ = true;
    }

    bool check_standstill() {
        if (!standstill_initialized_) {
            reset_standstill();
            return false;
        }
        double now = this->now().seconds();
        double dist_moved = std::hypot(robot_state_[0] - last_check_pos_.first, robot_state_[1] - last_check_pos_.second);

        if (dist_moved >= stuck_dist_threshold_) {
            last_moved_time_ = now;
            last_check_pos_ = {robot_state_[0], robot_state_[1]};
            return false;
        }

        double duration = now - last_moved_time_;
        if (duration >= stuck_time_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Standstill detected: %.1fs (dist=%.3fm)", duration, dist_moved);
            return true;
        }
        return false;
    }

    bool check_deviation() {
        if (smooth_path_.empty()) return false;
        double rx = robot_state_[0], ry = robot_state_[1];
        double min_d = std::numeric_limits<double>::max();

        for (const auto& p : smooth_path_) {
            double d = std::hypot(p.first - rx, p.second - ry);
            if (d < min_d) min_d = d;
        }

        if (min_d > replan_deviation_dist_) {
            RCLCPP_WARN(this->get_logger(), "Deviation: %.2fm > %.2fm", min_d, replan_deviation_dist_);
            return true;
        }
        return false;
    }

    void trigger_replan(const std::string& reason) {
        if (!replan_enabled_ || is_replanning_) return;
        if ((this->now().seconds() - last_replan_time_) < replan_cooldown_) return;

        if (!action_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(this->get_logger(), "ACO action server not available, skip replan.");
            return;
        }

        is_replanning_ = true;
        last_replan_time_ = this->now().seconds();
        reset_standstill();
        state_ = PlannerState::REPLANNING;

        RCLCPP_INFO(this->get_logger(), "Triggering replan [%s]...", reason.c_str());

        auto goal_msg = AcoPlan::Goal();
        goal_msg.goal_pose = final_goal_pose_;

        auto send_goal_options = rclcpp_action::Client<AcoPlan>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](auto goal_handle) {
            if (!goal_handle) {
                RCLCPP_WARN(this->get_logger(), "Replan goal REJECTED.");
                is_replanning_ = false;
                state_ = PlannerState::FOLLOWING;
            }
        };
        send_goal_options.feedback_callback = [this](auto, const std::shared_ptr<const AcoPlan::Feedback> feedback) {
            RCLCPP_DEBUG(this->get_logger(), "Replan: iter=%d, best=%.2f", feedback->iteration, feedback->current_best_length);
        };
        send_goal_options.result_callback = [this](const auto& result) {
            is_replanning_ = false;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED && !result.result->path.poses.empty()) {
                RCLCPP_INFO(this->get_logger(), "Replan success! length=%.2f", result.result->total_length);
            } else {
                RCLCPP_WARN(this->get_logger(), "Replan returned empty path.");
                state_ = PlannerState::FOLLOWING;
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // ================================================================
    // DWA LOGIC
    // ================================================================

    bool run_dwa(double tx, double ty) {
        double cur_v = robot_state_[3];
        double cur_w = robot_state_[4];
        
        double min_v_window = std::max(min_v_, cur_v - max_accel_ * dt_);
        double max_v_window = std::min(max_v_, cur_v + max_accel_ * dt_);
        double min_w_window = std::max(-max_w_, cur_w - max_dyaw_ * dt_);
        double max_w_window = std::min(max_w_, cur_w + max_dyaw_ * dt_);

        double rx = robot_state_[0], ry = robot_state_[1], rtheta = robot_state_[2];
        std::vector<Trajectory> candidates;

        for (double v = min_v_window; v <= max_v_window; v += v_res_) {
            for (double w = min_w_window; w <= max_w_window; w += w_res_) {
                Trajectory traj;
                traj.v = v; traj.w = w;
                traj.xs.reserve(num_steps_); traj.ys.reserve(num_steps_);

                double theta = rtheta;
                double x = rx; double y = ry;

                for (int i = 0; i < num_steps_; ++i) {
                    theta += w * dt_;
                    x += v * std::cos(theta) * dt_;
                    y += v * std::sin(theta) * dt_;
                    traj.xs.push_back(x);
                    traj.ys.push_back(y);
                }

                int traj_cost = check_trajectory_collision(traj.xs, traj.ys);
                if (traj_cost >= lethal_cost_) continue;

                double end_x = traj.xs.back();
                double end_y = traj.ys.back();
                double end_theta = rtheta + w * predict_time_; // approx

                traj.c_obs = static_cast<double>(lethal_cost_ - traj_cost);

                double error_angle = normalize_angle(std::atan2(ty - end_y, tx - end_x) - end_theta);
                traj.c_head = M_PI - std::abs(error_angle);
                traj.c_vel = std::abs(v);
                traj.c_ghead = traj.c_head; // Theo Python code delta_ghead giống c_head
                
                traj.c_dynobs = compute_dist_dynamic(traj.xs, traj.ys);
                if (traj.c_dynobs <= dynobs_lethal_) continue;

                candidates.push_back(traj);
            }
        }

        geometry_msgs::msg::Twist cmd;
        if (candidates.empty()) {
            RCLCPP_WARN(this->get_logger(), "DWA: Không có trajectory hợp lệ! Dừng robot.");
            cmd_vel_pub_->publish(cmd);
            return false;
        }

        // --- Score Normalization (thay thế NumPy ma trận) ---
        double min_c_head = 1e9, max_c_head = -1e9;
        double min_c_obs = 1e9, max_c_obs = -1e9;
        double min_c_vel = 1e9, max_c_vel = -1e9;
        double min_c_ghead = 1e9, max_c_ghead = -1e9;
        double min_c_dyn = 1e9, max_c_dyn = -1e9;

        for (const auto& c : candidates) {
            if(c.c_head < min_c_head) min_c_head = c.c_head; if(c.c_head > max_c_head) max_c_head = c.c_head;
            if(c.c_obs < min_c_obs) min_c_obs = c.c_obs; if(c.c_obs > max_c_obs) max_c_obs = c.c_obs;
            if(c.c_vel < min_c_vel) min_c_vel = c.c_vel; if(c.c_vel > max_c_vel) max_c_vel = c.c_vel;
            if(c.c_ghead < min_c_ghead) min_c_ghead = c.c_ghead; if(c.c_ghead > max_c_ghead) max_c_ghead = c.c_ghead;
            if(c.c_dynobs < min_c_dyn) min_c_dyn = c.c_dynobs; if(c.c_dynobs > max_c_dyn) max_c_dyn = c.c_dynobs;
        }

        auto normalize = [](double val, double min_v, double max_v) {
            double range = max_v - min_v;
            return (range > 1e-9) ? (val - min_v) / range : 1.0;
        };

        int best_idx = -1;
        double best_score = -1e9;

        for (size_t i = 0; i < candidates.size(); ++i) {
            double s_head = normalize(candidates[i].c_head, min_c_head, max_c_head);
            double s_obs = normalize(candidates[i].c_obs, min_c_obs, max_c_obs);
            double s_vel = normalize(candidates[i].c_vel, min_c_vel, max_c_vel);
            double s_ghead = normalize(candidates[i].c_ghead, min_c_ghead, max_c_ghead);
            double s_dyn = normalize(candidates[i].c_dynobs, min_c_dyn, max_c_dyn);

            double score = p_head_ * s_head + p_dist_ * s_obs + p_vel_ * s_vel + p_ghead_ * s_ghead + p_dynobs_ * s_dyn;

            if (score > best_score) {
                best_score = score;
                best_idx = static_cast<int>(i);
            }
        }

        cmd.linear.x = candidates[best_idx].v;
        cmd.angular.z = candidates[best_idx].w;

        if (candidates[best_idx].c_dynobs < 0.3) {
            RCLCPP_WARN(this->get_logger(), "dist_dyn=%.3f — robot gần vật cản động!", candidates[best_idx].c_dynobs);
        }

        visualize_trajectories(candidates, best_idx);
        cmd_vel_pub_->publish(cmd);
        return true;
    }

    void visualize_trajectories(const std::vector<Trajectory>& cands, int best_idx) {
        visualization_msgs::msg::MarkerArray msg;
        visualization_msgs::msg::Marker del_m;
        del_m.action = visualization_msgs::msg::Marker::DELETEALL;
        msg.markers.push_back(del_m);

        int step = 3;
        for (size_t i = 0; i < cands.size(); ++i) {
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

            for (size_t j = 0; j < cands[i].xs.size(); ++j) {
                geometry_msgs::msg::Point p;
                p.x = cands[i].xs[j]; p.y = cands[i].ys[j];
                m.points.push_back(p);
            }
            msg.markers.push_back(m);
        }
        traj_pub_->publish(msg);
    }

    // ================================================================
    // CONTROL LOOP
    // ================================================================

    void control_loop() {
        if (!update_pose_from_tf()) return;

        if (state_ == PlannerState::ROTATING) {
            if (waypoints_.empty()) return;
            auto target = (waypoints_.size() > 1) ? waypoints_[1] : waypoints_[0];
            double rx = robot_state_[0], ry = robot_state_[1], ryaw = robot_state_[2];
            double yaw_err = normalize_angle(std::atan2(target.second - ry, target.first - rx) - ryaw);
            
            geometry_msgs::msg::Twist cmd;
            if (std::abs(yaw_err) < yaw_tol_) {
                RCLCPP_INFO(this->get_logger(), "Aligned -> FOLLOWING");
                state_ = PlannerState::FOLLOWING;
            } else {
                double w = rot_kp_ * yaw_err;
                if (std::abs(w) < min_w_) w = min_w_ * (w > 0 ? 1 : -1);
                cmd.angular.z = std::clamp(w, -max_w_, max_w_);
            }
            cmd_vel_pub_->publish(cmd);

        } else if (state_ == PlannerState::FOLLOWING || state_ == PlannerState::REPLANNING) {
            if (waypoints_.empty()) {
                cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
                return;
            }

            // Update progress
            while (current_wp_idx_ < static_cast<int>(waypoints_.size()) - 1) {
                auto wp = waypoints_[current_wp_idx_];
                double rx = robot_state_[0], ry = robot_state_[1];
                double dist = std::hypot(rx - wp.first, ry - wp.second);
                double wp_tol = goal_tol_ * 2.0;

                auto next_wp = waypoints_[current_wp_idx_ + 1];
                double seg_x = next_wp.first - wp.first, seg_y = next_wp.second - wp.second;
                double rob_x = rx - wp.first, rob_y = ry - wp.second;
                double dot = seg_x * rob_x + seg_y * rob_y;
                double seg_len_sq = seg_x * seg_x + seg_y * seg_y;
                double t = (seg_len_sq > 0) ? (dot / seg_len_sq) : 0.0;

                if (dist < wp_tol || t > 0.0) {
                    current_wp_idx_++;
                } else {
                    break;
                }
            }

            auto final_wp = waypoints_.back();
            double dist_to_goal = std::hypot(robot_state_[0] - final_wp.first, robot_state_[1] - final_wp.second);

            if (dist_to_goal < goal_tol_) {
                cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
                RCLCPP_INFO(this->get_logger(), "GOAL REACHED!");
                
                visualization_msgs::msg::MarkerArray msg;
                visualization_msgs::msg::Marker del_m;
                del_m.action = visualization_msgs::msg::Marker::DELETEALL;
                msg.markers.push_back(del_m);
                traj_pub_->publish(msg);

                state_ = PlannerState::IDLE;
                waypoints_.clear();
                smooth_path_.clear();
                current_wp_idx_ = 0;
                standstill_initialized_ = false;
                return;
            }

            if (dist_to_goal < goal_tol_ * 2.0) {
                geometry_msgs::msg::Twist cmd;
                double rx = robot_state_[0], ry = robot_state_[1], ryaw = robot_state_[2];
                double yaw_err = normalize_angle(std::atan2(final_wp.second - ry, final_wp.first - rx) - ryaw);
                cmd.linear.x = std::min(max_v_ * 0.4, dist_to_goal * 0.8);
                cmd.angular.z = std::clamp(rot_kp_ * yaw_err, -max_w_ * 0.5, max_w_ * 0.5);
                cmd_vel_pub_->publish(cmd);
                return;
            }

            if (current_wp_idx_ < static_cast<int>(waypoints_.size())) {
                auto local_goal = waypoints_[current_wp_idx_];
                geometry_msgs::msg::PoseStamped goal_msg;
                goal_msg.header.frame_id = global_frame_;
                goal_msg.pose.position.x = local_goal.first;
                goal_msg.pose.position.y = local_goal.second;
                local_goal_pub_->publish(goal_msg);

                run_dwa(local_goal.first, local_goal.second);

                if (state_ == PlannerState::FOLLOWING && replan_enabled_) {
                    if (check_standstill()) trigger_replan("standstill");
                    else if (check_deviation()) trigger_replan("path deviation");
                }
            }

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