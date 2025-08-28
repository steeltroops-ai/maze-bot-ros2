#ifndef AUTONOMOUS_PATH_PLANNER_HPP
#define AUTONOMOUS_PATH_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>
#include <queue>
#include <unordered_map>
#include <memory>
#include <chrono>

namespace maze_bot_navigation
{

// Grid cell structure for A* algorithm
struct GridCell
{
    int x, y;
    double f_cost, g_cost, h_cost;
    bool is_obstacle;
    bool is_visited;
    GridCell* parent;
    
    GridCell() : x(0), y(0), f_cost(0), g_cost(0), h_cost(0), 
                 is_obstacle(false), is_visited(false), parent(nullptr) {}
    
    GridCell(int x_, int y_) : x(x_), y(y_), f_cost(0), g_cost(0), h_cost(0),
                               is_obstacle(false), is_visited(false), parent(nullptr) {}
    
    bool operator<(const GridCell& other) const {
        return f_cost > other.f_cost; // For priority queue (min-heap)
    }
};

// Dynamic obstacle representation
struct DynamicObstacle
{
    double x, y;
    double radius;
    double velocity_x, velocity_y;
    std::chrono::steady_clock::time_point last_update;
    double confidence;
    
    DynamicObstacle(double x_, double y_, double r) 
        : x(x_), y(y_), radius(r), velocity_x(0), velocity_y(0), 
          last_update(std::chrono::steady_clock::now()), confidence(1.0) {}
};

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

// Path planning result
struct PlanningResult
{
    std::vector<Waypoint> path;
    double total_cost;
    double planning_time_ms;
    bool success;
    std::string failure_reason;
    
    PlanningResult() : total_cost(0), planning_time_ms(0), success(false) {}
};

class AutonomousPathPlanner : public rclcpp::Node
{
public:
    AutonomousPathPlanner();
    ~AutonomousPathPlanner();
    
    // Main planning interface
    PlanningResult planPath(const geometry_msgs::msg::PoseStamped& start,
                           const geometry_msgs::msg::PoseStamped& goal);
    
    // Real-time path adaptation
    bool adaptPath(const std::vector<DynamicObstacle>& obstacles);
    
    // Get current optimal path
    std::vector<Waypoint> getCurrentPath() const { return current_path_; }
    
    // Check if path is valid
    bool isPathValid() const;
    
    // Emergency path planning
    PlanningResult emergencyReplan(const geometry_msgs::msg::PoseStamped& current_pose);

private:
    // Core A* algorithm implementation
    std::vector<GridCell*> aStarSearch(GridCell* start, GridCell* goal);
    
    // Heuristic functions
    double calculateHeuristic(const GridCell* current, const GridCell* goal);
    double calculateMovementCost(const GridCell* from, const GridCell* to);
    
    // Grid management
    void initializeGrid(double width, double height, double resolution);
    void updateGridFromLaserScan(const sensor_msgs::msg::LaserScan& scan);
    void updateGridFromDynamicObstacles(const std::vector<DynamicObstacle>& obstacles);
    GridCell* getGridCell(int x, int y);
    bool isValidGridCell(int x, int y);
    
    // Path optimization
    std::vector<Waypoint> optimizePath(const std::vector<GridCell*>& raw_path);
    std::vector<Waypoint> smoothPath(const std::vector<Waypoint>& path);
    void calculatePathCurvature(std::vector<Waypoint>& path);
    void assignTargetSpeeds(std::vector<Waypoint>& path);
    
    // Collision checking
    bool isCollisionFree(const Waypoint& from, const Waypoint& to);
    bool isPointSafe(double x, double y, double safety_margin = 0.3);
    double getObstacleDistance(double x, double y);
    
    // Dynamic obstacle tracking
    void updateDynamicObstacles(const sensor_msgs::msg::LaserScan& scan);
    void predictObstacleMovement(std::vector<DynamicObstacle>& obstacles, double time_horizon);
    
    // Coordinate transformations
    void worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y);
    void gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y);
    
    // ROS2 callbacks
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    // Visualization
    void publishPath(const std::vector<Waypoint>& path);
    void publishOccupancyGrid();
    void publishDynamicObstacles();
    void publishPlanningMarkers();
    
    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    // Visualization publisher removed for compatibility
    
    rclcpp::TimerBase::SharedPtr planning_timer_;
    
    // Planning parameters
    struct PlanningParams
    {
        double grid_resolution;
        double grid_width;
        double grid_height;
        double robot_radius;
        double safety_margin;
        double max_planning_time_ms;
        double path_smoothing_factor;
        double curvature_weight;
        double speed_weight;
        double obstacle_cost_multiplier;
        bool use_dynamic_obstacles;
        double dynamic_obstacle_prediction_time;
    } params_;
    
    // State variables
    std::vector<std::vector<GridCell>> grid_;
    std::vector<Waypoint> current_path_;
    std::vector<DynamicObstacle> dynamic_obstacles_;
    
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped current_goal_;
    
    bool grid_initialized_;
    bool has_current_goal_;
    
    // Performance monitoring
    struct PerformanceMetrics
    {
        double avg_planning_time_ms;
        double success_rate;
        int total_planning_requests;
        int successful_plans;
        double avg_path_length;
        double avg_path_optimality;
    } metrics_;
    
    // Thread safety
    std::mutex grid_mutex_;
    std::mutex path_mutex_;
    std::mutex obstacles_mutex_;
};

// Utility functions
double euclideanDistance(double x1, double y1, double x2, double y2);
double normalizeAngle(double angle);
bool isLineOfSight(double x1, double y1, double x2, double y2, 
                   const std::vector<std::vector<GridCell>>& grid);

} // namespace maze_bot_navigation

#endif // AUTONOMOUS_PATH_PLANNER_HPP
