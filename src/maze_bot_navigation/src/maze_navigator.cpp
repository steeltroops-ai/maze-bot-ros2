#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include <vector>

class MazeNavigator : public rclcpp::Node
{
public:
    MazeNavigator() : Node("maze_navigator")
    {
        // Declare parameters
        this->declare_parameter("goal_x", 4.0);
        this->declare_parameter("goal_y", 4.0);
        this->declare_parameter("goal_tolerance", 0.5);
        this->declare_parameter("max_linear_speed", 0.5);
        this->declare_parameter("max_angular_speed", 1.0);
        this->declare_parameter("wall_distance", 0.7);
        this->declare_parameter("front_distance", 0.8);
        this->declare_parameter("kp_wall", 2.0);
        this->declare_parameter("kd_wall", 0.5);
        
        // Get parameters
        goal_x_ = this->get_parameter("goal_x").as_double();
        goal_y_ = this->get_parameter("goal_y").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        wall_distance_ = this->get_parameter("wall_distance").as_double();
        front_distance_ = this->get_parameter("front_distance").as_double();
        kp_wall_ = this->get_parameter("kp_wall").as_double();
        kd_wall_ = this->get_parameter("kd_wall").as_double();

        // Initialize variables
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        previous_wall_error_ = 0.0;
        state_ = NavigationState::EXPLORE;
        goal_reached_ = false;

        // Initialize Bug2 algorithm variables
        start_x_ = 0.0;
        start_y_ = 0.0;
        on_m_line_ = true;
        closest_distance_to_goal_ = std::numeric_limits<double>::max();
        recovery_counter_ = 0;
        last_progress_time_ = 0.0;

        // Initialize enhanced navigation variables
        vision_target_detected_ = false;
        vision_target_x_ = 0.0;
        vision_target_y_ = 0.0;
        vision_target_confidence_ = 0.0;
        desired_wall_distance_ = 0.4;
        wall_following_speed_factor_ = 1.0;
        emergency_stop_distance_ = 0.15;
        emergency_stop_active_ = false;
        path_efficiency_ = 0.0;
        collision_count_ = 0;
        total_path_length_ = 0.0;
        navigation_start_time_ = this->now();

        // Create subscribers
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&MazeNavigator::laserCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&MazeNavigator::odomCallback, this, std::placeholders::_1));

        // Vision target subscriber
        vision_target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/vision/target_position", 10,
            std::bind(&MazeNavigator::visionTargetCallback, this, std::placeholders::_1));

        // Create publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Create timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&MazeNavigator::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Maze Navigator initialized");
        RCLCPP_INFO(this->get_logger(), "Goal: (%.2f, %.2f)", goal_x_, goal_y_);
    }

private:
    enum class NavigationState {
        EXPLORE,
        WALL_FOLLOW,
        GOAL_SEEK,
        RECOVERY,
        TURNING,
        STOPPED
    };

    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr vision_target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double goal_x_, goal_y_, goal_tolerance_;
    double max_linear_speed_, max_angular_speed_;
    double wall_distance_, front_distance_;
    double kp_wall_, kd_wall_;

    // State variables
    double current_x_, current_y_, current_yaw_;
    double previous_wall_error_;
    NavigationState state_;
    bool goal_reached_;

    // Bug2 algorithm variables
    double start_x_, start_y_;
    double m_line_slope_, m_line_intercept_;
    bool on_m_line_;
    double closest_distance_to_goal_;
    std::vector<std::pair<double, double>> hit_points_;
    int recovery_counter_;
    double last_progress_time_;

    // Enhanced navigation variables
    bool vision_target_detected_;
    double vision_target_x_, vision_target_y_;
    double vision_target_confidence_;

    // Improved wall following
    std::vector<double> wall_distance_history_;
    double desired_wall_distance_;
    double wall_following_speed_factor_;

    // Sensor fusion and safety
    double emergency_stop_distance_;
    bool emergency_stop_active_;
    std::vector<double> safety_zone_distances_;

    // Performance monitoring
    double path_efficiency_;
    int collision_count_;
    double total_path_length_;
    rclcpp::Time navigation_start_time_;

    // Sensor data
    std::vector<float> laser_ranges_;
    double min_front_distance_;
    double right_wall_distance_;
    double left_wall_distance_;

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        laser_ranges_ = msg->ranges;
        
        if (laser_ranges_.empty()) return;

        int num_readings = laser_ranges_.size();
        
        // Calculate front distance (average of front 30 degrees)
        int front_start = num_readings * 0.42;  // -30 degrees
        int front_end = num_readings * 0.58;    // +30 degrees
        
        min_front_distance_ = std::numeric_limits<double>::max();
        for (int i = front_start; i < front_end; ++i) {
            if (std::isfinite(laser_ranges_[i])) {
                min_front_distance_ = std::min(min_front_distance_, static_cast<double>(laser_ranges_[i]));
            }
        }

        // Calculate right wall distance (average of right 30 degrees)
        int right_start = num_readings * 0.75;  // -90 degrees
        int right_end = num_readings * 0.92;    // -60 degrees
        
        right_wall_distance_ = 0.0;
        int right_count = 0;
        for (int i = right_start; i < right_end; ++i) {
            if (std::isfinite(laser_ranges_[i])) {
                right_wall_distance_ += laser_ranges_[i];
                right_count++;
            }
        }
        if (right_count > 0) {
            right_wall_distance_ /= right_count;
        } else {
            right_wall_distance_ = std::numeric_limits<double>::max();
        }

        // Calculate left wall distance (average of left 30 degrees)
        int left_start = num_readings * 0.08;   // +60 degrees
        int left_end = num_readings * 0.25;     // +90 degrees
        
        left_wall_distance_ = 0.0;
        int left_count = 0;
        for (int i = left_start; i < left_end; ++i) {
            if (std::isfinite(laser_ranges_[i])) {
                left_wall_distance_ += laser_ranges_[i];
                left_count++;
            }
        }
        if (left_count > 0) {
            left_wall_distance_ /= left_count;
        } else {
            left_wall_distance_ = std::numeric_limits<double>::max();
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Convert quaternion to yaw
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
    }

    void controlLoop()
    {
        if (goal_reached_) {
            publishStop();
            return;
        }

        // Check if goal is reached
        double distance_to_goal = std::sqrt(
            std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
        
        if (distance_to_goal < goal_tolerance_) {
            goal_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            publishStop();
            return;
        }

        // Safety check - emergency stop if too close to front obstacle
        if (min_front_distance_ < 0.3) {
            RCLCPP_WARN(this->get_logger(), "Emergency stop - obstacle too close!");
            publishStop();
            return;
        }

        // Bug2 Algorithm Implementation
        geometry_msgs::msg::Twist cmd_vel;

        // Initialize M-line on first run
        if (start_x_ == 0.0 && start_y_ == 0.0) {
            start_x_ = current_x_;
            start_y_ = current_y_;
            calculateMLine();
        }

        switch (state_) {
            case NavigationState::EXPLORE:
                exploreBehavior(cmd_vel);
                break;
            case NavigationState::WALL_FOLLOW:
                wallFollowingBehavior(cmd_vel);
                break;
            case NavigationState::GOAL_SEEK:
                goalSeekingBehavior(cmd_vel);
                break;
            case NavigationState::RECOVERY:
                recoveryBehavior(cmd_vel);
                break;
            case NavigationState::TURNING:
                turningBehavior(cmd_vel);
                break;
            case NavigationState::STOPPED:
                publishStop();
                return;
        }

        cmd_vel_pub_->publish(cmd_vel);
    }

    void calculateMLine()
    {
        // Calculate M-line from start to goal
        if (std::abs(goal_x_ - start_x_) < 1e-6) {
            // Vertical line
            m_line_slope_ = std::numeric_limits<double>::infinity();
            m_line_intercept_ = start_x_;
        } else {
            m_line_slope_ = (goal_y_ - start_y_) / (goal_x_ - start_x_);
            m_line_intercept_ = start_y_ - m_line_slope_ * start_x_;
        }
    }

    bool isOnMLine(double x, double y, double tolerance = 0.3)
    {
        if (std::isinf(m_line_slope_)) {
            return std::abs(x - m_line_intercept_) < tolerance;
        } else {
            double expected_y = m_line_slope_ * x + m_line_intercept_;
            return std::abs(y - expected_y) < tolerance;
        }
    }

    bool hasLineOfSight()
    {
        // Ray-casting implementation for line-of-sight verification
        double angle_to_goal = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
        double relative_angle = angle_to_goal - current_yaw_;

        // Normalize angle
        while (relative_angle > M_PI) relative_angle -= 2 * M_PI;
        while (relative_angle < -M_PI) relative_angle += 2 * M_PI;

        // Check if goal is roughly in front
        if (std::abs(relative_angle) > M_PI/3) return false;

        // Check laser scan in direction of goal
        int num_readings = laser_ranges_.size();
        if (num_readings == 0) return false;

        // Convert relative angle to laser scan index
        int goal_index = static_cast<int>((relative_angle + M_PI) / (2 * M_PI) * num_readings);
        goal_index = std::max(0, std::min(num_readings - 1, goal_index));

        // Check a cone around the goal direction
        int cone_width = 10;
        for (int i = -cone_width; i <= cone_width; ++i) {
            int idx = goal_index + i;
            if (idx >= 0 && idx < num_readings && std::isfinite(laser_ranges_[idx])) {
                if (laser_ranges_[idx] < distance_to_goal() * 0.9) {
                    return false; // Obstacle in the way
                }
            }
        }
        return true;
    }

    void exploreBehavior(geometry_msgs::msg::Twist& cmd_vel)
    {
        // Move towards goal until obstacle is encountered
        if (min_front_distance_ < front_distance_) {
            // Hit an obstacle, record hit point and switch to wall following
            hit_points_.push_back({current_x_, current_y_});
            state_ = NavigationState::WALL_FOLLOW;
            on_m_line_ = false;
            return;
        }

        // Check if we can see the goal
        if (hasLineOfSight() && distance_to_goal() < 3.0) {
            state_ = NavigationState::GOAL_SEEK;
            return;
        }

        // Move towards goal
        double angle_to_goal = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
        double angle_error = angle_to_goal - current_yaw_;

        // Normalize angle
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        cmd_vel.linear.x = max_linear_speed_ * 0.8;
        cmd_vel.angular.z = std::max(-max_angular_speed_,
                                   std::min(max_angular_speed_, 2.0 * angle_error));
    }

    void recoveryBehavior(geometry_msgs::msg::Twist& cmd_vel)
    {
        // Recovery behavior: rotate in place to find new path
        recovery_counter_++;

        if (recovery_counter_ < 50) {
            // Rotate left
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = max_angular_speed_ * 0.5;
        } else if (recovery_counter_ < 100) {
            // Rotate right
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -max_angular_speed_ * 0.5;
        } else {
            // Move backward
            cmd_vel.linear.x = -max_linear_speed_ * 0.3;
            cmd_vel.angular.z = 0.0;

            if (recovery_counter_ > 150) {
                recovery_counter_ = 0;
                state_ = NavigationState::EXPLORE;
            }
        }

        // Check if recovery is successful
        if (min_front_distance_ > front_distance_ * 1.5) {
            recovery_counter_ = 0;
            state_ = NavigationState::EXPLORE;
        }
    }

    void visionTargetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        vision_target_detected_ = true;
        vision_target_x_ = msg->point.x;
        vision_target_y_ = msg->point.y;

        // Calculate confidence based on distance and angle
        double distance = std::sqrt(vision_target_x_ * vision_target_x_ + vision_target_y_ * vision_target_y_);
        double angle = std::atan2(vision_target_y_, vision_target_x_);

        // Higher confidence for targets that are closer and more centered
        vision_target_confidence_ = std::exp(-distance / 5.0) * std::exp(-std::abs(angle) / 1.0);

        RCLCPP_DEBUG(this->get_logger(), "Vision target detected at (%.2f, %.2f) with confidence %.2f",
                    vision_target_x_, vision_target_y_, vision_target_confidence_);
    }

    void wallFollowingBehavior(geometry_msgs::msg::Twist& cmd_vel)
    {
        // Bug2 wall following with M-line checking
        double wall_error = wall_distance_ - right_wall_distance_;
        double wall_derivative = wall_error - previous_wall_error_;
        previous_wall_error_ = wall_error;

        // PD controller for wall following
        double angular_correction = kp_wall_ * wall_error + kd_wall_ * wall_derivative;

        // Check for front obstacle
        if (min_front_distance_ < front_distance_) {
            state_ = NavigationState::TURNING;
            return;
        }

        // Set velocities
        cmd_vel.linear.x = max_linear_speed_ * 0.6;  // Reduced speed for better control
        cmd_vel.angular.z = std::max(-max_angular_speed_,
                                   std::min(max_angular_speed_, angular_correction));

        // Bug2 algorithm: Check if we're back on M-line and closer to goal
        if (isOnMLine(current_x_, current_y_)) {
            double current_distance = distance_to_goal();
            if (current_distance < closest_distance_to_goal_) {
                closest_distance_to_goal_ = current_distance;
                on_m_line_ = true;

                // Check if we can see the goal
                if (hasLineOfSight()) {
                    state_ = NavigationState::GOAL_SEEK;
                } else {
                    state_ = NavigationState::EXPLORE;
                }
            }
        }

        // Stuck detection
        static double last_x = current_x_;
        static double last_y = current_y_;
        static int stuck_counter = 0;

        if (std::abs(current_x_ - last_x) < 0.05 && std::abs(current_y_ - last_y) < 0.05) {
            stuck_counter++;
            if (stuck_counter > 100) {  // 10 seconds at 10Hz
                state_ = NavigationState::RECOVERY;
                stuck_counter = 0;
            }
        } else {
            stuck_counter = 0;
        }

        last_x = current_x_;
        last_y = current_y_;
    }

    void turningBehavior(geometry_msgs::msg::Twist& cmd_vel)
    {
        // Turn left when hitting a wall
        cmd_vel.linear.x = 0.1;
        cmd_vel.angular.z = max_angular_speed_ * 0.8;

        // Switch back to wall following when front is clear
        if (min_front_distance_ > front_distance_ * 1.2) {
            state_ = NavigationState::WALL_FOLLOW;
        }
    }

    void goalSeekingBehavior(geometry_msgs::msg::Twist& cmd_vel)
    {
        // Enhanced goal seeking with potential field method
        double angle_to_goal = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
        double angle_error = angle_to_goal - current_yaw_;

        // Normalize angle
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        // Check for obstacles in path to goal
        if (min_front_distance_ < front_distance_ || !hasLineOfSight()) {
            state_ = NavigationState::WALL_FOLLOW;
            return;
        }

        // Potential field calculation
        double attractive_force_x = (goal_x_ - current_x_) / distance_to_goal();
        double attractive_force_y = (goal_y_ - current_y_) / distance_to_goal();

        // Repulsive forces from obstacles
        double repulsive_force_x = 0.0;
        double repulsive_force_y = 0.0;

        if (min_front_distance_ < 2.0) {
            double repulsive_strength = 1.0 / (min_front_distance_ * min_front_distance_);
            repulsive_force_x = -repulsive_strength * std::cos(current_yaw_);
            repulsive_force_y = -repulsive_strength * std::sin(current_yaw_);
        }

        // Combine forces
        double total_force_x = attractive_force_x + repulsive_force_x;
        double total_force_y = attractive_force_y + repulsive_force_y;

        // Calculate desired heading
        double desired_angle = std::atan2(total_force_y, total_force_x);
        double final_angle_error = desired_angle - current_yaw_;

        // Normalize angle
        while (final_angle_error > M_PI) final_angle_error -= 2 * M_PI;
        while (final_angle_error < -M_PI) final_angle_error += 2 * M_PI;

        // Move towards goal with potential field guidance
        cmd_vel.linear.x = max_linear_speed_ * 0.9;
        cmd_vel.angular.z = std::max(-max_angular_speed_,
                                   std::min(max_angular_speed_, 2.5 * final_angle_error));
    }



    double distance_to_goal()
    {
        return std::sqrt(std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
    }

    void publishStop()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeNavigator>());
    rclcpp::shutdown();
    return 0;
}
