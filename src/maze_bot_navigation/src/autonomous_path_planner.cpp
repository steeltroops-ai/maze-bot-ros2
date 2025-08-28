#include "maze_bot_navigation/autonomous_path_planner.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>

namespace maze_bot_navigation
{

AutonomousPathPlanner::AutonomousPathPlanner()
    : Node("autonomous_path_planner"),
      grid_initialized_(false),
      has_current_goal_(false)
{
    // Initialize planning parameters
    params_.grid_resolution = 0.05;  // 5cm resolution
    params_.grid_width = 20.0;       // 20m x 20m grid
    params_.grid_height = 20.0;
    params_.robot_radius = 0.25;     // 25cm robot radius
    params_.safety_margin = 0.15;    // 15cm safety margin
    params_.max_planning_time_ms = 100.0;  // 100ms max planning time
    params_.path_smoothing_factor = 0.3;
    params_.curvature_weight = 0.2;
    params_.speed_weight = 0.3;
    params_.obstacle_cost_multiplier = 10.0;
    params_.use_dynamic_obstacles = true;
    params_.dynamic_obstacle_prediction_time = 2.0;  // 2 second prediction
    
    // Initialize performance metrics
    metrics_.avg_planning_time_ms = 0.0;
    metrics_.success_rate = 0.0;
    metrics_.total_planning_requests = 0;
    metrics_.successful_plans = 0;
    metrics_.avg_path_length = 0.0;
    metrics_.avg_path_optimality = 0.0;
    
    // Initialize grid
    initializeGrid(params_.grid_width, params_.grid_height, params_.grid_resolution);
    
    // ROS2 subscribers
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&AutonomousPathPlanner::laserScanCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&AutonomousPathPlanner::odometryCallback, this, std::placeholders::_1));
    
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10, std::bind(&AutonomousPathPlanner::goalCallback, this, std::placeholders::_1));
    
    // ROS2 publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_grid", 10);
    
    RCLCPP_INFO(this->get_logger(), "Autonomous Path Planner initialized with %dx%d grid at %.3fm resolution",
                static_cast<int>(params_.grid_width / params_.grid_resolution),
                static_cast<int>(params_.grid_height / params_.grid_resolution),
                params_.grid_resolution);
}

AutonomousPathPlanner::~AutonomousPathPlanner()
{
    RCLCPP_INFO(this->get_logger(), "Path Planner Performance Summary:");
    RCLCPP_INFO(this->get_logger(), "  Success Rate: %.1f%% (%d/%d)", 
                metrics_.success_rate * 100, metrics_.successful_plans, metrics_.total_planning_requests);
    RCLCPP_INFO(this->get_logger(), "  Avg Planning Time: %.2fms", metrics_.avg_planning_time_ms);
    RCLCPP_INFO(this->get_logger(), "  Avg Path Optimality: %.2f", metrics_.avg_path_optimality);
}

void AutonomousPathPlanner::initializeGrid(double width, double height, double resolution)
{
    std::lock_guard<std::mutex> lock(grid_mutex_);
    
    int grid_width = static_cast<int>(width / resolution);
    int grid_height = static_cast<int>(height / resolution);
    
    grid_.clear();
    grid_.resize(grid_height);
    
    for (int y = 0; y < grid_height; ++y) {
        grid_[y].resize(grid_width);
        for (int x = 0; x < grid_width; ++x) {
            grid_[y][x] = GridCell(x, y);
        }
    }
    
    grid_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Grid initialized: %dx%d cells", grid_width, grid_height);
}

PlanningResult AutonomousPathPlanner::planPath(const geometry_msgs::msg::PoseStamped& start,
                                              const geometry_msgs::msg::PoseStamped& goal)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    PlanningResult result;
    
    metrics_.total_planning_requests++;
    
    if (!grid_initialized_) {
        result.failure_reason = "Grid not initialized";
        return result;
    }
    
    // Convert world coordinates to grid coordinates
    int start_x, start_y, goal_x, goal_y;
    worldToGrid(start.pose.position.x, start.pose.position.y, start_x, start_y);
    worldToGrid(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);
    
    // Validate start and goal positions
    if (!isValidGridCell(start_x, start_y) || !isValidGridCell(goal_x, goal_y)) {
        result.failure_reason = "Start or goal position outside grid bounds";
        return result;
    }
    
    if (!isPointSafe(goal.pose.position.x, goal.pose.position.y)) {
        result.failure_reason = "Goal position is not safe (too close to obstacles)";
        return result;
    }
    
    // Reset grid for new search
    std::lock_guard<std::mutex> lock(grid_mutex_);
    for (auto& row : grid_) {
        for (auto& cell : row) {
            cell.f_cost = cell.g_cost = cell.h_cost = 0;
            cell.is_visited = false;
            cell.parent = nullptr;
        }
    }
    
    // Perform A* search
    GridCell* start_cell = &grid_[start_y][start_x];
    GridCell* goal_cell = &grid_[goal_y][goal_x];
    
    std::vector<GridCell*> raw_path = aStarSearch(start_cell, goal_cell);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    result.planning_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    if (raw_path.empty()) {
        result.failure_reason = "No path found by A* algorithm";
        return result;
    }
    
    // Optimize and smooth the path
    result.path = optimizePath(raw_path);
    result.path = smoothPath(result.path);
    calculatePathCurvature(result.path);
    assignTargetSpeeds(result.path);
    
    // Calculate path metrics
    result.total_cost = 0;
    for (size_t i = 1; i < result.path.size(); ++i) {
        result.total_cost += euclideanDistance(
            result.path[i-1].x, result.path[i-1].y,
            result.path[i].x, result.path[i].y);
    }
    
    result.success = true;
    metrics_.successful_plans++;
    
    // Update performance metrics
    metrics_.avg_planning_time_ms = (metrics_.avg_planning_time_ms * (metrics_.total_planning_requests - 1) + 
                                    result.planning_time_ms) / metrics_.total_planning_requests;
    metrics_.success_rate = static_cast<double>(metrics_.successful_plans) / metrics_.total_planning_requests;
    
    // Store current path
    std::lock_guard<std::mutex> path_lock(path_mutex_);
    current_path_ = result.path;
    
    RCLCPP_DEBUG(this->get_logger(), "Path planned successfully: %.1fms, %zu waypoints, %.2fm total length",
                result.planning_time_ms, result.path.size(), result.total_cost);
    
    return result;
}

std::vector<GridCell*> AutonomousPathPlanner::aStarSearch(GridCell* start, GridCell* goal)
{
    std::priority_queue<GridCell> open_set;
    std::unordered_map<GridCell*, bool> in_open_set;
    
    // Initialize start cell
    start->g_cost = 0;
    start->h_cost = calculateHeuristic(start, goal);
    start->f_cost = start->g_cost + start->h_cost;
    
    open_set.push(*start);
    in_open_set[start] = true;
    
    // 8-directional movement
    const int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    while (!open_set.empty()) {
        // Check timeout
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(current_time - start_time).count();
        if (elapsed_ms > params_.max_planning_time_ms) {
            RCLCPP_WARN(this->get_logger(), "A* search timeout after %.1fms", elapsed_ms);
            break;
        }
        
        // Get cell with lowest f_cost
        GridCell current = open_set.top();
        open_set.pop();
        
        GridCell* current_ptr = getGridCell(current.x, current.y);
        if (!current_ptr || current_ptr->is_visited) continue;
        
        current_ptr->is_visited = true;
        in_open_set[current_ptr] = false;
        
        // Check if we reached the goal
        if (current_ptr == goal) {
            std::vector<GridCell*> path;
            GridCell* cell = goal;
            while (cell != nullptr) {
                path.push_back(cell);
                cell = cell->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        // Explore neighbors
        for (int i = 0; i < 8; ++i) {
            int next_x = current_ptr->x + dx[i];
            int next_y = current_ptr->y + dy[i];
            
            if (!isValidGridCell(next_x, next_y)) continue;
            
            GridCell* neighbor = getGridCell(next_x, next_y);
            if (!neighbor || neighbor->is_visited || neighbor->is_obstacle) continue;
            
            double tentative_g_cost = current_ptr->g_cost + calculateMovementCost(current_ptr, neighbor);
            
            if (in_open_set.find(neighbor) == in_open_set.end() || tentative_g_cost < neighbor->g_cost) {
                neighbor->parent = current_ptr;
                neighbor->g_cost = tentative_g_cost;
                neighbor->h_cost = calculateHeuristic(neighbor, goal);
                neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
                
                if (in_open_set.find(neighbor) == in_open_set.end()) {
                    open_set.push(*neighbor);
                    in_open_set[neighbor] = true;
                }
            }
        }
    }
    
    return std::vector<GridCell*>(); // No path found
}

double AutonomousPathPlanner::calculateHeuristic(const GridCell* current, const GridCell* goal)
{
    // Euclidean distance heuristic
    double dx = (goal->x - current->x) * params_.grid_resolution;
    double dy = (goal->y - current->y) * params_.grid_resolution;
    return std::sqrt(dx * dx + dy * dy);
}

double AutonomousPathPlanner::calculateMovementCost(const GridCell* from, const GridCell* to)
{
    double base_cost = euclideanDistance(
        from->x * params_.grid_resolution, from->y * params_.grid_resolution,
        to->x * params_.grid_resolution, to->y * params_.grid_resolution);
    
    // Add obstacle proximity cost
    double world_x, world_y;
    gridToWorld(to->x, to->y, world_x, world_y);
    double obstacle_distance = getObstacleDistance(world_x, world_y);
    
    if (obstacle_distance < params_.safety_margin) {
        base_cost *= params_.obstacle_cost_multiplier;
    }
    
    return base_cost;
}

void AutonomousPathPlanner::worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y)
{
    // Assuming grid center is at (0,0) in world coordinates
    grid_x = static_cast<int>((world_x + params_.grid_width / 2.0) / params_.grid_resolution);
    grid_y = static_cast<int>((world_y + params_.grid_height / 2.0) / params_.grid_resolution);
}

void AutonomousPathPlanner::gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y)
{
    world_x = (grid_x * params_.grid_resolution) - params_.grid_width / 2.0;
    world_y = (grid_y * params_.grid_resolution) - params_.grid_height / 2.0;
}

GridCell* AutonomousPathPlanner::getGridCell(int x, int y)
{
    if (!isValidGridCell(x, y)) return nullptr;
    return &grid_[y][x];
}

bool AutonomousPathPlanner::isValidGridCell(int x, int y)
{
    return x >= 0 && x < static_cast<int>(grid_[0].size()) && 
           y >= 0 && y < static_cast<int>(grid_.size());
}

// Utility functions
double euclideanDistance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

std::vector<Waypoint> AutonomousPathPlanner::optimizePath(const std::vector<GridCell*>& raw_path)
{
    std::vector<Waypoint> waypoints;
    if (raw_path.empty()) return waypoints;

    // Convert grid cells to waypoints
    for (const auto& cell : raw_path) {
        double world_x, world_y;
        gridToWorld(cell->x, cell->y, world_x, world_y);
        waypoints.emplace_back(world_x, world_y);
    }

    // Line-of-sight optimization (remove unnecessary waypoints)
    std::vector<Waypoint> optimized;
    if (waypoints.empty()) return optimized;

    optimized.push_back(waypoints[0]);

    for (size_t i = 1; i < waypoints.size() - 1; ++i) {
        const Waypoint& prev = optimized.back();
        const Waypoint& next = waypoints[i + 1];

        // Check if we can go directly from prev to next
        if (!isCollisionFree(prev, next)) {
            optimized.push_back(waypoints[i]);
        }
    }

    if (waypoints.size() > 1) {
        optimized.push_back(waypoints.back());
    }

    return optimized;
}

std::vector<Waypoint> AutonomousPathPlanner::smoothPath(const std::vector<Waypoint>& path)
{
    if (path.size() < 3) return path;

    std::vector<Waypoint> smoothed = path;

    // Apply smoothing iterations
    for (int iter = 0; iter < 3; ++iter) {
        for (size_t i = 1; i < smoothed.size() - 1; ++i) {
            double new_x = smoothed[i].x + params_.path_smoothing_factor *
                          (smoothed[i-1].x + smoothed[i+1].x - 2 * smoothed[i].x);
            double new_y = smoothed[i].y + params_.path_smoothing_factor *
                          (smoothed[i-1].y + smoothed[i+1].y - 2 * smoothed[i].y);

            // Only apply smoothing if the new position is collision-free
            if (isPointSafe(new_x, new_y)) {
                smoothed[i].x = new_x;
                smoothed[i].y = new_y;
            }
        }
    }

    return smoothed;
}

void AutonomousPathPlanner::calculatePathCurvature(std::vector<Waypoint>& path)
{
    if (path.size() < 3) return;

    for (size_t i = 1; i < path.size() - 1; ++i) {
        // Calculate curvature using three consecutive points
        double x1 = path[i-1].x, y1 = path[i-1].y;
        double x2 = path[i].x, y2 = path[i].y;
        double x3 = path[i+1].x, y3 = path[i+1].y;

        // Calculate vectors
        double v1x = x2 - x1, v1y = y2 - y1;
        double v2x = x3 - x2, v2y = y3 - y2;

        // Calculate angle change
        double angle1 = std::atan2(v1y, v1x);
        double angle2 = std::atan2(v2y, v2x);
        double angle_diff = normalizeAngle(angle2 - angle1);

        // Calculate arc length
        double dist1 = euclideanDistance(x1, y1, x2, y2);
        double dist2 = euclideanDistance(x2, y2, x3, y3);
        double arc_length = (dist1 + dist2) / 2.0;

        // Curvature = angle change / arc length
        path[i].curvature = std::abs(angle_diff) / std::max(arc_length, 0.01);
    }

    // Set curvature for first and last points
    if (path.size() >= 2) {
        path[0].curvature = path[1].curvature;
        path.back().curvature = path[path.size()-2].curvature;
    }
}

void AutonomousPathPlanner::assignTargetSpeeds(std::vector<Waypoint>& path)
{
    const double max_speed = 1.0;  // 1 m/s maximum speed
    const double min_speed = 0.2;  // 0.2 m/s minimum speed
    const double curvature_speed_factor = 0.5;

    for (auto& waypoint : path) {
        // Base speed calculation based on curvature
        double curvature_speed = max_speed / (1.0 + curvature_speed_factor * waypoint.curvature);

        // Apply safety margin based on obstacle proximity
        double obstacle_dist = getObstacleDistance(waypoint.x, waypoint.y);
        double safety_speed = max_speed * std::min(1.0, obstacle_dist / (2.0 * params_.safety_margin));

        // Take minimum of curvature and safety speeds
        waypoint.target_speed = std::max(min_speed, std::min(curvature_speed, safety_speed));
    }
}

bool AutonomousPathPlanner::isCollisionFree(const Waypoint& from, const Waypoint& to)
{
    const int num_checks = 20;
    for (int i = 0; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;
        double x = from.x + t * (to.x - from.x);
        double y = from.y + t * (to.y - from.y);

        if (!isPointSafe(x, y)) {
            return false;
        }
    }
    return true;
}

bool AutonomousPathPlanner::isPointSafe(double x, double y, double safety_margin)
{
    // Check grid bounds
    int grid_x, grid_y;
    worldToGrid(x, y, grid_x, grid_y);
    if (!isValidGridCell(grid_x, grid_y)) return false;

    // Check static obstacles in grid
    std::lock_guard<std::mutex> lock(grid_mutex_);

    // Check area around the point
    int margin_cells = static_cast<int>((params_.robot_radius + safety_margin) / params_.grid_resolution) + 1;

    for (int dy = -margin_cells; dy <= margin_cells; ++dy) {
        for (int dx = -margin_cells; dx <= margin_cells; ++dx) {
            int check_x = grid_x + dx;
            int check_y = grid_y + dy;

            if (!isValidGridCell(check_x, check_y)) continue;

            if (grid_[check_y][check_x].is_obstacle) {
                double check_world_x, check_world_y;
                gridToWorld(check_x, check_y, check_world_x, check_world_y);
                double dist = euclideanDistance(x, y, check_world_x, check_world_y);

                if (dist < params_.robot_radius + safety_margin) {
                    return false;
                }
            }
        }
    }

    // Check dynamic obstacles
    std::lock_guard<std::mutex> obs_lock(obstacles_mutex_);
    for (const auto& obstacle : dynamic_obstacles_) {
        double dist = euclideanDistance(x, y, obstacle.x, obstacle.y);
        if (dist < obstacle.radius + params_.robot_radius + safety_margin) {
            return false;
        }
    }

    return true;
}

double AutonomousPathPlanner::getObstacleDistance(double x, double y)
{
    double min_distance = std::numeric_limits<double>::max();

    // Check static obstacles
    std::lock_guard<std::mutex> lock(grid_mutex_);
    int grid_x, grid_y;
    worldToGrid(x, y, grid_x, grid_y);

    int search_radius = static_cast<int>(2.0 / params_.grid_resolution); // Search within 2m

    for (int dy = -search_radius; dy <= search_radius; ++dy) {
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
            int check_x = grid_x + dx;
            int check_y = grid_y + dy;

            if (!isValidGridCell(check_x, check_y)) continue;

            if (grid_[check_y][check_x].is_obstacle) {
                double obs_world_x, obs_world_y;
                gridToWorld(check_x, check_y, obs_world_x, obs_world_y);
                double dist = euclideanDistance(x, y, obs_world_x, obs_world_y);
                min_distance = std::min(min_distance, dist);
            }
        }
    }

    // Check dynamic obstacles
    std::lock_guard<std::mutex> obs_lock(obstacles_mutex_);
    for (const auto& obstacle : dynamic_obstacles_) {
        double dist = euclideanDistance(x, y, obstacle.x, obstacle.y) - obstacle.radius;
        min_distance = std::min(min_distance, std::max(0.0, dist));
    }

    return min_distance;
}

void AutonomousPathPlanner::updateGridFromLaserScan(const sensor_msgs::msg::LaserScan& scan)
{
    std::lock_guard<std::mutex> lock(grid_mutex_);

    // Clear previous obstacle markings (keep some persistence)
    for (auto& row : grid_) {
        for (auto& cell : row) {
            if (cell.is_obstacle) {
                // Decay obstacle confidence over time
                cell.is_obstacle = false; // Reset for this update
            }
        }
    }

    // Get robot position
    double robot_x = current_pose_.pose.position.x;
    double robot_y = current_pose_.pose.position.y;
    double robot_yaw = 2 * std::atan2(current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);

    // Process laser scan
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double range = scan.ranges[i];

        // Skip invalid readings
        if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
            continue;
        }

        // Calculate beam angle
        double beam_angle = scan.angle_min + i * scan.angle_increment;
        double world_angle = robot_yaw + beam_angle;

        // Calculate obstacle position
        double obs_x = robot_x + range * std::cos(world_angle);
        double obs_y = robot_y + range * std::sin(world_angle);

        // Convert to grid coordinates
        int grid_x, grid_y;
        worldToGrid(obs_x, obs_y, grid_x, grid_y);

        // Mark obstacle in grid
        if (isValidGridCell(grid_x, grid_y)) {
            grid_[grid_y][grid_x].is_obstacle = true;

            // Also mark nearby cells to account for robot size
            int robot_radius_cells = static_cast<int>(params_.robot_radius / params_.grid_resolution) + 1;
            for (int dy = -robot_radius_cells; dy <= robot_radius_cells; ++dy) {
                for (int dx = -robot_radius_cells; dx <= robot_radius_cells; ++dx) {
                    int mark_x = grid_x + dx;
                    int mark_y = grid_y + dy;

                    if (isValidGridCell(mark_x, mark_y)) {
                        double dist = std::sqrt(dx*dx + dy*dy) * params_.grid_resolution;
                        if (dist <= params_.robot_radius) {
                            grid_[mark_y][mark_x].is_obstacle = true;
                        }
                    }
                }
            }
        }

        // Ray tracing for free space (mark cells between robot and obstacle as free)
        int robot_grid_x, robot_grid_y;
        worldToGrid(robot_x, robot_y, robot_grid_x, robot_grid_y);

        // Bresenham's line algorithm for ray tracing
        int dx = std::abs(grid_x - robot_grid_x);
        int dy = std::abs(grid_y - robot_grid_y);
        int x_step = (robot_grid_x < grid_x) ? 1 : -1;
        int y_step = (robot_grid_y < grid_y) ? 1 : -1;
        int error = dx - dy;

        int current_x = robot_grid_x;
        int current_y = robot_grid_y;

        while (current_x != grid_x || current_y != grid_y) {
            if (isValidGridCell(current_x, current_y)) {
                // Don't override obstacle markings
                if (!grid_[current_y][current_x].is_obstacle) {
                    // This cell is free space
                }
            }

            int error2 = 2 * error;
            if (error2 > -dy) {
                error -= dy;
                current_x += x_step;
            }
            if (error2 < dx) {
                error += dx;
                current_y += y_step;
            }
        }
    }
}

void AutonomousPathPlanner::updateDynamicObstacles(const sensor_msgs::msg::LaserScan& /* scan */)
{
    std::lock_guard<std::mutex> lock(obstacles_mutex_);

    // Simple dynamic obstacle detection (can be enhanced with tracking)
    // For now, detect obstacles that are moving based on position changes

    auto current_time = std::chrono::steady_clock::now();

    // Age existing obstacles
    for (auto& obstacle : dynamic_obstacles_) {
        auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - obstacle.last_update).count();

        if (time_diff > 2000) { // Remove obstacles older than 2 seconds
            obstacle.confidence *= 0.9;
        }
    }

    // Remove low-confidence obstacles
    dynamic_obstacles_.erase(
        std::remove_if(dynamic_obstacles_.begin(), dynamic_obstacles_.end(),
                      [](const DynamicObstacle& obs) { return obs.confidence < 0.3; }),
        dynamic_obstacles_.end());
}

void AutonomousPathPlanner::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!grid_initialized_) return;

    // Update static obstacle grid
    updateGridFromLaserScan(*msg);

    // Update dynamic obstacles
    updateDynamicObstacles(*msg);

    // Publish updated occupancy grid
    publishOccupancyGrid();

    // Check if current path needs replanning
    if (has_current_goal_ && !isPathValid()) {
        RCLCPP_INFO(this->get_logger(), "Current path invalid, replanning...");
        auto result = planPath(current_pose_, current_goal_);
        if (result.success) {
            publishPath(result.path);
        }
    }
}

void AutonomousPathPlanner::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
}

void AutonomousPathPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_goal_ = *msg;
    has_current_goal_ = true;

    RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)",
                msg->pose.position.x, msg->pose.position.y);

    // Plan path to new goal
    auto result = planPath(current_pose_, current_goal_);
    if (result.success) {
        publishPath(result.path);
        RCLCPP_INFO(this->get_logger(), "Path planned successfully: %zu waypoints, %.2fm length",
                    result.path.size(), result.total_cost);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Path planning failed: %s", result.failure_reason.c_str());
    }
}

bool AutonomousPathPlanner::isPathValid() const
{
    // Note: Cannot use mutex in const method, assume thread safety handled externally
    if (current_path_.empty()) return false;

    // Simple validation - check if path exists
    // Full collision checking would require non-const methods
    return !current_path_.empty();
}

void AutonomousPathPlanner::publishPath(const std::vector<Waypoint>& path)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    for (const auto& waypoint : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = waypoint.x;
        pose.pose.position.y = waypoint.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;

        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
}

void AutonomousPathPlanner::publishOccupancyGrid()
{
    std::lock_guard<std::mutex> lock(grid_mutex_);

    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = this->now();
    grid_msg.header.frame_id = "map";

    grid_msg.info.resolution = params_.grid_resolution;
    grid_msg.info.width = grid_[0].size();
    grid_msg.info.height = grid_.size();
    grid_msg.info.origin.position.x = -params_.grid_width / 2.0;
    grid_msg.info.origin.position.y = -params_.grid_height / 2.0;
    grid_msg.info.origin.orientation.w = 1.0;

    grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height);

    for (size_t y = 0; y < grid_.size(); ++y) {
        for (size_t x = 0; x < grid_[y].size(); ++x) {
            int index = y * grid_msg.info.width + x;
            if (grid_[y][x].is_obstacle) {
                grid_msg.data[index] = 100; // Occupied
            } else {
                grid_msg.data[index] = 0;   // Free
            }
        }
    }

    grid_pub_->publish(grid_msg);
}

// Removed publishPlanningMarkers method

} // namespace maze_bot_navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<maze_bot_navigation::AutonomousPathPlanner>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in autonomous path planner: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
