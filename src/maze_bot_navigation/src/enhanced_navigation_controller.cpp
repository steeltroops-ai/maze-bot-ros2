#include "maze_bot_navigation/enhanced_navigation_controller.hpp"
#include <algorithm>
#include <cmath>

// Utility function
double euclideanDistance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

namespace maze_bot_navigation
{

EnhancedNavigationController::EnhancedNavigationController()
    : Node("enhanced_navigation_controller"),
      current_mode_(NavigationMode::IDLE),
      previous_mode_(NavigationMode::IDLE),
      current_waypoint_index_(0),
      has_active_goal_(false),
      path_completed_(false),
      has_laser_data_(false),
      vision_target_detected_(false),
      vision_target_x_(0.0),
      vision_target_y_(0.0),
      vision_target_confidence_(0.0),
      total_path_length_(0.0),
      traveled_distance_(0.0)
{
    // Initialize performance metrics
    metrics_.success_rate = 0.0;
    metrics_.average_completion_time = 0.0;
    metrics_.path_efficiency = 0.0;
    metrics_.collision_count = 0.0;
    metrics_.emergency_stops = 0.0;
    metrics_.total_distance_traveled = 0.0;
    metrics_.mission_start_time = std::chrono::steady_clock::now();
    
    // Path planner integration removed for simplicity
    
    // ROS2 subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&EnhancedNavigationController::odometryCallback, this, std::placeholders::_1));
    
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&EnhancedNavigationController::laserScanCallback, this, std::placeholders::_1));
    
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path", 10, std::bind(&EnhancedNavigationController::pathCallback, this, std::placeholders::_1));
    
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10, std::bind(&EnhancedNavigationController::goalCallback, this, std::placeholders::_1));
    
    // Vision subscription removed for simplicity
    
    // ROS2 publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
    progress_pub_ = this->create_publisher<std_msgs::msg::Float64>("/navigation_progress", 10);
    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_target", 10);
    
    // Control timer (20Hz for smooth control)
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&EnhancedNavigationController::controlTimerCallback, this));
    
    // Diagnostics timer (1Hz)
    diagnostics_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&EnhancedNavigationController::publishDiagnostics, this));
    
    RCLCPP_INFO(this->get_logger(), "Enhanced Navigation Controller initialized");
    RCLCPP_INFO(this->get_logger(), "  Max linear speed: %.2f m/s", path_controller_.max_linear_speed);
    RCLCPP_INFO(this->get_logger(), "  Max angular speed: %.2f rad/s", path_controller_.max_angular_speed);
    RCLCPP_INFO(this->get_logger(), "  Emergency stop distance: %.2f m", safety_system_.emergency_stop_distance);
}

EnhancedNavigationController::~EnhancedNavigationController()
{
    // Emergency stop on shutdown
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    
    RCLCPP_INFO(this->get_logger(), "Enhanced Navigation Controller shutdown");
    publishPerformanceMetrics();
}

void EnhancedNavigationController::controlTimerCallback()
{
    // Update vehicle state
    updateVehicleState();
    
    // Update safety system
    updateSafetySystem();
    
    // Update navigation mode based on current conditions
    updateNavigationMode();
    
    // Calculate and publish control command
    geometry_msgs::msg::Twist cmd_vel = calculateControlCommand();
    
    // Apply safety limits and smoothing
    applyAccelerationLimits(cmd_vel);
    smoothControlCommand(cmd_vel);
    
    // Publish command
    cmd_vel_pub_->publish(cmd_vel);
    
    // Update performance metrics
    updateVehicleState();
}

geometry_msgs::msg::Twist EnhancedNavigationController::calculateControlCommand()
{
    geometry_msgs::msg::Twist cmd_vel;
    
    switch (current_mode_) {
        case NavigationMode::IDLE:
            // Do nothing, return zero velocity
            break;
            
        case NavigationMode::PATH_FOLLOWING:
            cmd_vel = pathFollowingControl();
            break;
            
        case NavigationMode::GOAL_SEEKING:
            cmd_vel = goalSeekingControl();
            break;
            
        case NavigationMode::OBSTACLE_AVOIDANCE:
            cmd_vel = obstacleAvoidanceControl();
            break;
            
        case NavigationMode::VISION_GUIDED:
            cmd_vel = visionGuidedControl();
            break;
            
        case NavigationMode::EMERGENCY_STOP:
            cmd_vel = emergencyControl();
            break;
            
        case NavigationMode::RECOVERY:
            // Implement recovery behavior (backing up, rotating)
            cmd_vel.linear.x = -0.2;  // Slow backward movement
            cmd_vel.angular.z = 0.5;  // Gentle rotation
            break;
    }
    
    return cmd_vel;
}

geometry_msgs::msg::Twist EnhancedNavigationController::pathFollowingControl()
{
    if (current_path_.empty()) {
        return geometry_msgs::msg::Twist();
    }
    
    // Use pure pursuit algorithm for path following
    return purePursuitControl();
}

geometry_msgs::msg::Twist EnhancedNavigationController::purePursuitControl()
{
    geometry_msgs::msg::Twist cmd_vel;
    
    // Find lookahead point
    int lookahead_index = findLookaheadPoint();
    if (lookahead_index < 0 || lookahead_index >= static_cast<int>(current_path_.size())) {
        return cmd_vel;
    }
    
    const Waypoint& target = current_path_[lookahead_index];
    
    // Calculate curvature to target
    double curvature = calculateCurvature(target);
    
    // Calculate target speed based on curvature and obstacles
    double target_speed = calculateTargetSpeed();
    target_speed = applySpeedLimits(target_speed);
    
    // Apply pure pursuit control law
    cmd_vel.linear.x = target_speed;
    cmd_vel.angular.z = curvature * target_speed;
    
    // Limit angular velocity
    cmd_vel.angular.z = std::max(-path_controller_.max_angular_speed,
                                std::min(path_controller_.max_angular_speed, cmd_vel.angular.z));
    
    return cmd_vel;
}

int EnhancedNavigationController::findLookaheadPoint()
{
    if (current_path_.empty()) return -1;
    
    // Calculate dynamic lookahead distance based on speed
    double speed = vehicle_state_.linear_velocity;
    double dynamic_lookahead = path_controller_.min_lookahead + 
                              path_controller_.lookahead_ratio * speed;
    dynamic_lookahead = std::max(path_controller_.min_lookahead,
                                std::min(path_controller_.max_lookahead, dynamic_lookahead));
    
    // Find the point on the path that is closest to the lookahead distance
    double min_distance_diff = std::numeric_limits<double>::max();
    int best_index = current_waypoint_index_;
    
    for (int i = current_waypoint_index_; i < static_cast<int>(current_path_.size()); ++i) {
        double distance = euclideanDistance(
            vehicle_state_.x, vehicle_state_.y,
            current_path_[i].x, current_path_[i].y);
        
        double distance_diff = std::abs(distance - dynamic_lookahead);
        if (distance_diff < min_distance_diff) {
            min_distance_diff = distance_diff;
            best_index = i;
        }
        
        // If we've gone past the lookahead distance, stop searching
        if (distance > dynamic_lookahead * 1.5) break;
    }
    
    return best_index;
}

double EnhancedNavigationController::calculateCurvature(const Waypoint& target)
{
    // Calculate curvature using pure pursuit formula
    double dx = target.x - vehicle_state_.x;
    double dy = target.y - vehicle_state_.y;
    
    // Transform to vehicle coordinate system
    double cos_yaw = std::cos(vehicle_state_.yaw);
    double sin_yaw = std::sin(vehicle_state_.yaw);
    
    double local_x = dx * cos_yaw + dy * sin_yaw;
    double local_y = -dx * sin_yaw + dy * cos_yaw;
    
    // Calculate lookahead distance
    double lookahead_dist = std::sqrt(local_x * local_x + local_y * local_y);
    
    if (lookahead_dist < 0.1) return 0.0; // Avoid division by zero
    
    // Pure pursuit curvature formula
    double curvature = 2.0 * local_y / (lookahead_dist * lookahead_dist);
    
    return curvature;
}

double EnhancedNavigationController::calculateTargetSpeed()
{
    if (current_path_.empty() || current_waypoint_index_ >= static_cast<int>(current_path_.size())) {
        return 0.0;
    }
    
    // Base speed from waypoint
    double base_speed = current_path_[current_waypoint_index_].target_speed;
    
    // Reduce speed based on curvature
    double curvature = current_path_[current_waypoint_index_].curvature;
    double curvature_speed = path_controller_.max_linear_speed / 
                            (1.0 + path_controller_.curvature_speed_factor * curvature);
    
    // Reduce speed based on obstacle proximity
    double obstacle_speed = path_controller_.max_linear_speed;
    if (has_laser_data_) {
        double min_distance = std::numeric_limits<double>::max();
        for (const auto& range : latest_scan_.ranges) {
            if (std::isfinite(range)) {
                min_distance = std::min(min_distance, static_cast<double>(range));
            }
        }
        
        if (min_distance < safety_system_.warning_distance) {
            obstacle_speed = path_controller_.max_linear_speed * 
                           (min_distance / safety_system_.warning_distance) * 
                           path_controller_.obstacle_speed_factor;
        }
    }
    
    // Take minimum of all speed constraints
    return std::min({base_speed, curvature_speed, obstacle_speed});
}

double EnhancedNavigationController::applySpeedLimits(double target_speed)
{
    return std::max(0.0, std::min(path_controller_.max_linear_speed, target_speed));
}

bool EnhancedNavigationController::checkCollisionRisk()
{
    if (!has_laser_data_) return false;
    
    // Check for immediate collision risk
    for (const auto& range : latest_scan_.ranges) {
        if (std::isfinite(range) && range < safety_system_.emergency_stop_distance) {
            return true;
        }
    }
    
    return false;
}

void EnhancedNavigationController::updateNavigationMode()
{
    NavigationMode new_mode = current_mode_;
    
    // Emergency stop has highest priority
    if (checkCollisionRisk()) {
        new_mode = NavigationMode::EMERGENCY_STOP;
    }
    // Vision guidance when target is detected with high confidence
    else if (vision_target_detected_ && vision_target_confidence_ > 0.8) {
        new_mode = NavigationMode::VISION_GUIDED;
    }
    // Path following when we have a valid path
    else if (!current_path_.empty() && has_active_goal_) {
        new_mode = NavigationMode::PATH_FOLLOWING;
    }
    // Goal seeking when we have a goal but no path
    else if (has_active_goal_) {
        new_mode = NavigationMode::GOAL_SEEKING;
    }
    // Idle when no active navigation
    else {
        new_mode = NavigationMode::IDLE;
    }
    
    if (new_mode != current_mode_) {
        transitionToMode(new_mode);
    }
}

void EnhancedNavigationController::transitionToMode(NavigationMode new_mode)
{
    RCLCPP_INFO(this->get_logger(), "Navigation mode transition: %d -> %d", 
                static_cast<int>(current_mode_), static_cast<int>(new_mode));
    
    previous_mode_ = current_mode_;
    current_mode_ = new_mode;
    
    // Handle mode-specific initialization
    switch (new_mode) {
        case NavigationMode::EMERGENCY_STOP:
            safety_system_.emergency_active = true;
            safety_system_.last_emergency_time = std::chrono::steady_clock::now();
            metrics_.emergency_stops++;
            break;
            
        case NavigationMode::PATH_FOLLOWING:
            navigation_start_time_ = std::chrono::steady_clock::now();
            break;
            
        case NavigationMode::IDLE:
            if (previous_mode_ == NavigationMode::PATH_FOLLOWING) {
                // Navigation completed
                auto completion_time = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - navigation_start_time_).count();
                metrics_.average_completion_time = completion_time;
            }
            break;
            
        default:
            break;
    }
}

void EnhancedNavigationController::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Update vehicle state
    vehicle_state_.x = msg->pose.pose.position.x;
    vehicle_state_.y = msg->pose.pose.position.y;

    // Convert quaternion to yaw
    double qw = msg->pose.pose.orientation.w;
    double qz = msg->pose.pose.orientation.z;
    vehicle_state_.yaw = 2.0 * std::atan2(qz, qw);

    // Update velocities
    vehicle_state_.linear_velocity = msg->twist.twist.linear.x;
    vehicle_state_.angular_velocity = msg->twist.twist.angular.z;

    vehicle_state_.timestamp = std::chrono::steady_clock::now();

    // Update traveled distance
    if (previous_vehicle_state_.timestamp != std::chrono::steady_clock::time_point{}) {
        double dt = std::chrono::duration<double>(
            vehicle_state_.timestamp - previous_vehicle_state_.timestamp).count();
        double distance_increment = vehicle_state_.linear_velocity * dt;
        traveled_distance_ += std::abs(distance_increment);
        metrics_.total_distance_traveled += std::abs(distance_increment);
    }

    previous_vehicle_state_ = vehicle_state_;
}

void EnhancedNavigationController::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    latest_scan_ = *msg;
    has_laser_data_ = true;
}

void EnhancedNavigationController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(path_mutex_);

    // Convert Path message to Waypoint vector
    current_path_.clear();
    for (const auto& pose : msg->poses) {
        Waypoint waypoint(pose.pose.position.x, pose.pose.position.y);
        current_path_.push_back(waypoint);
    }

    current_waypoint_index_ = 0;
    path_completed_ = false;

    // Calculate total path length
    total_path_length_ = 0.0;
    for (size_t i = 1; i < current_path_.size(); ++i) {
        total_path_length_ += euclideanDistance(
            current_path_[i-1].x, current_path_[i-1].y,
            current_path_[i].x, current_path_[i].y);
    }

    RCLCPP_INFO(this->get_logger(), "New path received: %zu waypoints, %.2fm total length",
                current_path_.size(), total_path_length_);
}

void EnhancedNavigationController::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_goal_ = *msg;
    has_active_goal_ = true;

    RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)",
                msg->pose.position.x, msg->pose.position.y);

    // Path planning integration removed - will use external path planner
}

// Vision callback removed for simplicity

geometry_msgs::msg::Twist EnhancedNavigationController::visionGuidedControl()
{
    geometry_msgs::msg::Twist cmd_vel;

    if (!vision_target_detected_) {
        return cmd_vel;
    }

    // Check if vision data is recent
    auto time_since_update = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_vision_update_).count();

    if (time_since_update > 500) { // 500ms timeout
        vision_target_detected_ = false;
        return cmd_vel;
    }

    // Calculate control commands to approach vision target
    double target_distance = std::sqrt(vision_target_x_ * vision_target_x_ + vision_target_y_ * vision_target_y_);
    double target_angle = std::atan2(vision_target_y_, vision_target_x_);

    // Proportional control for angular velocity
    cmd_vel.angular.z = 2.0 * target_angle;
    cmd_vel.angular.z = std::max(-path_controller_.max_angular_speed,
                                std::min(path_controller_.max_angular_speed, cmd_vel.angular.z));

    // Proportional control for linear velocity
    if (std::abs(target_angle) < 0.3) { // Only move forward if roughly aligned
        cmd_vel.linear.x = std::min(0.5, target_distance * 0.3);

        // Slow down as we approach the target
        if (target_distance < 1.0) {
            cmd_vel.linear.x *= (target_distance / 1.0);
        }
    }

    return cmd_vel;
}

geometry_msgs::msg::Twist EnhancedNavigationController::goalSeekingControl()
{
    geometry_msgs::msg::Twist cmd_vel;

    if (!has_active_goal_) {
        return cmd_vel;
    }

    // Simple goal seeking when no path is available
    double dx = current_goal_.pose.position.x - vehicle_state_.x;
    double dy = current_goal_.pose.position.y - vehicle_state_.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    double angle_to_goal = std::atan2(dy, dx);
    double angle_error = angle_to_goal - vehicle_state_.yaw;

    // Normalize angle error
    while (angle_error > M_PI) angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;

    // Check if goal is reached
    if (distance < 0.3) {
        has_active_goal_ = false;
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return cmd_vel;
    }

    // Proportional control
    cmd_vel.angular.z = 2.0 * angle_error;
    cmd_vel.angular.z = std::max(-path_controller_.max_angular_speed,
                                std::min(path_controller_.max_angular_speed, cmd_vel.angular.z));

    // Only move forward if roughly aligned with goal
    if (std::abs(angle_error) < 0.5) {
        cmd_vel.linear.x = std::min(0.8, distance * 0.5);

        // Apply obstacle-based speed reduction
        cmd_vel.linear.x = calculateSafeSpeed(cmd_vel.linear.x);
    }

    return cmd_vel;
}

geometry_msgs::msg::Twist EnhancedNavigationController::obstacleAvoidanceControl()
{
    geometry_msgs::msg::Twist cmd_vel;

    if (!has_laser_data_) {
        return cmd_vel;
    }

    // Simple reactive obstacle avoidance
    double left_distance = 0.0, right_distance = 0.0, front_distance = 0.0;
    int left_count = 0, right_count = 0, front_count = 0;

    size_t num_readings = latest_scan_.ranges.size();
    for (size_t i = 0; i < num_readings; ++i) {
        if (!std::isfinite(latest_scan_.ranges[i])) continue;

        double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;
        double range = latest_scan_.ranges[i];

        if (angle < -M_PI/4) { // Right side
            right_distance += range;
            right_count++;
        } else if (angle > M_PI/4) { // Left side
            left_distance += range;
            left_count++;
        } else { // Front
            front_distance += range;
            front_count++;
        }
    }

    // Calculate average distances
    if (left_count > 0) left_distance /= left_count;
    if (right_count > 0) right_distance /= right_count;
    if (front_count > 0) front_distance /= front_count;

    // Reactive control
    if (front_distance < 1.0) {
        // Turn away from closer side
        if (left_distance > right_distance) {
            cmd_vel.angular.z = 1.0; // Turn left
        } else {
            cmd_vel.angular.z = -1.0; // Turn right
        }
        cmd_vel.linear.x = 0.2; // Slow forward movement
    } else {
        cmd_vel.linear.x = 0.5; // Normal forward movement
    }

    return cmd_vel;
}

geometry_msgs::msg::Twist EnhancedNavigationController::emergencyControl()
{
    // Emergency stop - return zero velocity
    return geometry_msgs::msg::Twist();
}

double EnhancedNavigationController::calculateSafeSpeed(double desired_speed)
{
    if (!has_laser_data_) return desired_speed;

    double min_distance = std::numeric_limits<double>::max();
    for (const auto& range : latest_scan_.ranges) {
        if (std::isfinite(range)) {
            min_distance = std::min(min_distance, static_cast<double>(range));
        }
    }

    // Reduce speed based on closest obstacle
    if (min_distance < safety_system_.warning_distance) {
        double speed_factor = min_distance / safety_system_.warning_distance;
        return desired_speed * speed_factor * 0.5; // Additional safety factor
    }

    return desired_speed;
}

void EnhancedNavigationController::updateVehicleState()
{
    // Vehicle state is updated in odometry callback
    // This method can be used for additional state calculations
}

void EnhancedNavigationController::updateSafetySystem()
{
    // Check if emergency condition has cleared
    if (safety_system_.emergency_active) {
        auto time_since_emergency = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - safety_system_.last_emergency_time).count();

        if (time_since_emergency > 1000 && !checkCollisionRisk()) { // 1 second delay
            safety_system_.emergency_active = false;
            RCLCPP_INFO(this->get_logger(), "Emergency condition cleared");
        }
    }
}

void EnhancedNavigationController::applyAccelerationLimits(geometry_msgs::msg::Twist& cmd)
{
    // Apply acceleration limits for smooth control
    static double prev_linear = 0.0;
    static double prev_angular = 0.0;
    static auto prev_time = std::chrono::steady_clock::now();

    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - prev_time).count();

    if (dt > 0.001) { // Avoid division by very small numbers
        // Linear acceleration limit
        double linear_accel = (cmd.linear.x - prev_linear) / dt;
        if (std::abs(linear_accel) > path_controller_.max_linear_acceleration) {
            double sign = (linear_accel > 0) ? 1.0 : -1.0;
            cmd.linear.x = prev_linear + sign * path_controller_.max_linear_acceleration * dt;
        }

        // Angular acceleration limit
        double angular_accel = (cmd.angular.z - prev_angular) / dt;
        if (std::abs(angular_accel) > path_controller_.max_angular_acceleration) {
            double sign = (angular_accel > 0) ? 1.0 : -1.0;
            cmd.angular.z = prev_angular + sign * path_controller_.max_angular_acceleration * dt;
        }
    }

    prev_linear = cmd.linear.x;
    prev_angular = cmd.angular.z;
    prev_time = current_time;
}

void EnhancedNavigationController::smoothControlCommand(geometry_msgs::msg::Twist& cmd)
{
    // Apply low-pass filter for smoother control
    static double alpha = 0.8; // Smoothing factor
    static geometry_msgs::msg::Twist prev_cmd;

    cmd.linear.x = alpha * cmd.linear.x + (1.0 - alpha) * prev_cmd.linear.x;
    cmd.angular.z = alpha * cmd.angular.z + (1.0 - alpha) * prev_cmd.angular.z;

    prev_cmd = cmd;
}

void EnhancedNavigationController::publishDiagnostics()
{
    // Publish emergency status
    std_msgs::msg::Bool emergency_msg;
    emergency_msg.data = safety_system_.emergency_active;
    emergency_pub_->publish(emergency_msg);

    // Publish navigation progress
    std_msgs::msg::Float64 progress_msg;
    progress_msg.data = getPathProgress();
    progress_pub_->publish(progress_msg);

    // Publish current target
    if (!current_path_.empty() && current_waypoint_index_ < static_cast<int>(current_path_.size())) {
        geometry_msgs::msg::PoseStamped target_msg;
        target_msg.header.stamp = this->now();
        target_msg.header.frame_id = "map";
        target_msg.pose.position.x = current_path_[current_waypoint_index_].x;
        target_msg.pose.position.y = current_path_[current_waypoint_index_].y;
        target_msg.pose.orientation.w = 1.0;
        target_pub_->publish(target_msg);
    }
}

void EnhancedNavigationController::publishPerformanceMetrics()
{
    RCLCPP_INFO(this->get_logger(), "=== NAVIGATION PERFORMANCE METRICS ===");
    RCLCPP_INFO(this->get_logger(), "Total distance traveled: %.2f m", metrics_.total_distance_traveled);
    RCLCPP_INFO(this->get_logger(), "Emergency stops: %.0f", metrics_.emergency_stops);
    RCLCPP_INFO(this->get_logger(), "Average completion time: %.1f s", metrics_.average_completion_time);

    if (total_path_length_ > 0) {
        metrics_.path_efficiency = total_path_length_ / std::max(traveled_distance_, 0.1);
        RCLCPP_INFO(this->get_logger(), "Path efficiency: %.2f", metrics_.path_efficiency);
    }
}

double EnhancedNavigationController::getDistanceToGoal() const
{
    if (!has_active_goal_) return 0.0;

    return euclideanDistance(
        vehicle_state_.x, vehicle_state_.y,
        current_goal_.pose.position.x, current_goal_.pose.position.y);
}

double EnhancedNavigationController::getPathProgress() const
{
    if (current_path_.empty() || total_path_length_ <= 0) return 0.0;

    // Calculate progress based on distance traveled along path
    double progress_distance = 0.0;
    for (int i = 1; i <= current_waypoint_index_ && i < static_cast<int>(current_path_.size()); ++i) {
        progress_distance += euclideanDistance(
            current_path_[i-1].x, current_path_[i-1].y,
            current_path_[i].x, current_path_[i].y);
    }

    return progress_distance / total_path_length_;
}

void EnhancedNavigationController::setGoal(const geometry_msgs::msg::PoseStamped& goal)
{
    current_goal_ = goal;
    has_active_goal_ = true;
}

void EnhancedNavigationController::setPath(const std::vector<Waypoint>& path)
{
    std::lock_guard<std::mutex> lock(path_mutex_);
    current_path_ = path;
    current_waypoint_index_ = 0;
    path_completed_ = false;
}

void EnhancedNavigationController::emergencyStop()
{
    safety_system_.emergency_active = true;
    transitionToMode(NavigationMode::EMERGENCY_STOP);
}

void EnhancedNavigationController::resumeNavigation()
{
    if (safety_system_.emergency_active && !checkCollisionRisk()) {
        safety_system_.emergency_active = false;
        updateNavigationMode();
    }
}

} // namespace maze_bot_navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<maze_bot_navigation::EnhancedNavigationController>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in enhanced navigation controller: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
