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
        state_ = NavigationState::WALL_FOLLOWING;
        goal_reached_ = false;

        // Create subscribers
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&MazeNavigator::laserCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&MazeNavigator::odomCallback, this, std::placeholders::_1));

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
        WALL_FOLLOWING,
        TURNING,
        GOAL_SEEKING,
        STOPPED
    };

    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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

        // Main navigation logic
        geometry_msgs::msg::Twist cmd_vel;
        
        switch (state_) {
            case NavigationState::WALL_FOLLOWING:
                wallFollowingBehavior(cmd_vel);
                break;
            case NavigationState::TURNING:
                turningBehavior(cmd_vel);
                break;
            case NavigationState::GOAL_SEEKING:
                goalSeekingBehavior(cmd_vel);
                break;
            case NavigationState::STOPPED:
                publishStop();
                return;
        }

        cmd_vel_pub_->publish(cmd_vel);
    }

    void wallFollowingBehavior(geometry_msgs::msg::Twist& cmd_vel)
    {
        // Right-hand wall following
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
        cmd_vel.linear.x = max_linear_speed_ * 0.7;  // Reduced speed for better control
        cmd_vel.angular.z = std::max(-max_angular_speed_, 
                                   std::min(max_angular_speed_, angular_correction));

        // Check if we should switch to goal seeking
        if (distance_to_goal < 2.0 && canSeeGoal()) {
            state_ = NavigationState::GOAL_SEEKING;
        }
    }

    void turningBehavior(geometry_msgs::msg::Twist& cmd_vel)
    {
        // Turn left when hitting a wall
        cmd_vel.linear.x = 0.1;
        cmd_vel.angular.z = max_angular_speed_ * 0.8;

        // Switch back to wall following when front is clear
        if (min_front_distance_ > front_distance_ * 1.2) {
            state_ = NavigationState::WALL_FOLLOWING;
        }
    }

    void goalSeekingBehavior(geometry_msgs::msg::Twist& cmd_vel)
    {
        // Calculate angle to goal
        double angle_to_goal = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
        double angle_error = angle_to_goal - current_yaw_;
        
        // Normalize angle
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        // Check for obstacles in path to goal
        if (min_front_distance_ < front_distance_) {
            state_ = NavigationState::WALL_FOLLOWING;
            return;
        }

        // Move towards goal
        cmd_vel.linear.x = max_linear_speed_ * 0.8;
        cmd_vel.angular.z = std::max(-max_angular_speed_, 
                                   std::min(max_angular_speed_, 2.0 * angle_error));
    }

    bool canSeeGoal()
    {
        // Simple line-of-sight check (can be improved with ray tracing)
        double angle_to_goal = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
        double relative_angle = angle_to_goal - current_yaw_;
        
        // Normalize angle
        while (relative_angle > M_PI) relative_angle -= 2 * M_PI;
        while (relative_angle < -M_PI) relative_angle += 2 * M_PI;
        
        // Check if goal is roughly in front and path seems clear
        return (std::abs(relative_angle) < M_PI/4 && min_front_distance_ > 2.0);
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
