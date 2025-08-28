#ifndef ENHANCED_NAVIGATION_CONTROLLER_HPP
#define ENHANCED_NAVIGATION_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

// External dependencies removed for simplicity

#include <memory>
#include <vector>
#include <chrono>

namespace maze_bot_navigation
{

// Waypoint structure for path following
struct Waypoint
{
    double x, y;
    double target_speed;
    double curvature;
    double safety_margin;

    Waypoint(double x_, double y_, double speed = 0.5)
        : x(x_), y(y_), target_speed(speed), curvature(0.0), safety_margin(0.3) {}
};

// Control states for the navigation system
enum class NavigationMode
{
    IDLE,
    PATH_FOLLOWING,
    GOAL_SEEKING,
    OBSTACLE_AVOIDANCE,
    EMERGENCY_STOP,
    RECOVERY,
    VISION_GUIDED
};

// Vehicle control parameters
struct VehicleState
{
    double x, y, yaw;
    double linear_velocity, angular_velocity;
    double linear_acceleration, angular_acceleration;
    std::chrono::steady_clock::time_point timestamp;
    
    VehicleState() : x(0), y(0), yaw(0), linear_velocity(0), angular_velocity(0),
                     linear_acceleration(0), angular_acceleration(0),
                     timestamp(std::chrono::steady_clock::now()) {}
};

// Path following controller
struct PathFollowingController
{
    // Pure pursuit parameters
    double lookahead_distance;
    double min_lookahead;
    double max_lookahead;
    double lookahead_ratio;
    
    // Speed control
    double max_linear_speed;
    double max_angular_speed;
    double max_linear_acceleration;
    double max_angular_acceleration;
    
    // Curvature-based speed control
    double curvature_speed_factor;
    double obstacle_speed_factor;
    
    PathFollowingController() 
        : lookahead_distance(1.0), min_lookahead(0.5), max_lookahead(3.0), lookahead_ratio(0.3),
          max_linear_speed(1.0), max_angular_speed(1.5), max_linear_acceleration(2.0), 
          max_angular_acceleration(3.0), curvature_speed_factor(0.5), obstacle_speed_factor(0.3) {}
};

// Emergency and safety systems
struct SafetySystem
{
    double emergency_stop_distance;
    double warning_distance;
    double collision_prediction_time;
    double max_deceleration;
    bool emergency_active;
    std::chrono::steady_clock::time_point last_emergency_time;
    
    SafetySystem() 
        : emergency_stop_distance(0.3), warning_distance(0.8), collision_prediction_time(2.0),
          max_deceleration(3.0), emergency_active(false),
          last_emergency_time(std::chrono::steady_clock::now()) {}
};

class EnhancedNavigationController : public rclcpp::Node
{
public:
    EnhancedNavigationController();
    ~EnhancedNavigationController();
    
    // Main control interface
    void setGoal(const geometry_msgs::msg::PoseStamped& goal);
    void setPath(const std::vector<Waypoint>& path);
    void emergencyStop();
    void resumeNavigation();
    
    // Status queries
    NavigationMode getCurrentMode() const { return current_mode_; }
    bool isNavigating() const { return current_mode_ != NavigationMode::IDLE; }
    double getDistanceToGoal() const;
    double getPathProgress() const;
    
    // Performance metrics
    struct PerformanceMetrics
    {
        double success_rate;
        double average_completion_time;
        double path_efficiency;
        double collision_count;
        double emergency_stops;
        double total_distance_traveled;
        std::chrono::steady_clock::time_point mission_start_time;
    } metrics_;

private:
    // Core control methods
    geometry_msgs::msg::Twist calculateControlCommand();
    geometry_msgs::msg::Twist pathFollowingControl();
    geometry_msgs::msg::Twist goalSeekingControl();
    geometry_msgs::msg::Twist obstacleAvoidanceControl();
    geometry_msgs::msg::Twist visionGuidedControl();
    geometry_msgs::msg::Twist emergencyControl();
    
    // Path following algorithms
    geometry_msgs::msg::Twist purePursuitControl();
    geometry_msgs::msg::Twist stanleyControl();
    int findLookaheadPoint();
    double calculateCurvature(const Waypoint& target);
    
    // Speed control
    double calculateTargetSpeed();
    double applySpeedLimits(double target_speed);
    double calculateSafeSpeed(double obstacle_distance);
    
    // Safety and collision avoidance
    bool checkCollisionRisk();
    bool predictCollision(double prediction_time);
    geometry_msgs::msg::Twist calculateAvoidanceManeuver();
    void updateSafetySystem();
    
    // Vision integration
    void integrateVisionTarget();
    bool isVisionTargetValid();
    geometry_msgs::msg::Twist approachVisionTarget();
    
    // State management
    void updateNavigationMode();
    void transitionToMode(NavigationMode new_mode);
    bool shouldSwitchMode();
    
    // Vehicle dynamics
    void updateVehicleState();
    void applyAccelerationLimits(geometry_msgs::msg::Twist& cmd);
    void smoothControlCommand(geometry_msgs::msg::Twist& cmd);
    
    // ROS2 callbacks
    void controlTimerCallback();
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    // Vision callback removed for simplicity
    
    // Monitoring and diagnostics
    void publishDiagnostics();
    void publishPerformanceMetrics();
    void logNavigationEvent(const std::string& event);
    
    // ROS2 components
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    // Vision subscription removed for simplicity
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr progress_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    
    // Integrated components removed for simplicity
    
    // State variables
    NavigationMode current_mode_;
    NavigationMode previous_mode_;
    VehicleState vehicle_state_;
    VehicleState previous_vehicle_state_;
    
    // Control parameters
    PathFollowingController path_controller_;
    SafetySystem safety_system_;
    
    // Path and goal management
    std::vector<Waypoint> current_path_;
    geometry_msgs::msg::PoseStamped current_goal_;
    int current_waypoint_index_;
    bool has_active_goal_;
    bool path_completed_;
    
    // Sensor data
    sensor_msgs::msg::LaserScan latest_scan_;
    bool has_laser_data_;
    
    // Vision integration
    bool vision_target_detected_;
    double vision_target_x_, vision_target_y_;
    double vision_target_confidence_;
    std::chrono::steady_clock::time_point last_vision_update_;
    
    // Performance tracking
    std::chrono::steady_clock::time_point navigation_start_time_;
    double total_path_length_;
    double traveled_distance_;
    
    // Thread safety
    std::mutex state_mutex_;
    std::mutex path_mutex_;
    std::mutex sensor_mutex_;
};

} // namespace maze_bot_navigation

#endif // ENHANCED_NAVIGATION_CONTROLLER_HPP
