#!/usr/bin/env python3

"""
Basic Navigation System Test
Tests basic navigation functionality
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64
import time
import math
import json
import os
from datetime import datetime

class AutonomousNavigationTester(Node):
    def __init__(self):
        super().__init__('autonomous_navigation_tester')
        
        # Test configuration
        self.test_goals = [
            (4.0, 4.0),    # Standard goal - far corner
            (0.0, 4.0),    # North goal
            (-3.0, 0.0),   # West goal  
            (3.0, -3.0),   # Southeast goal
            (0.0, 0.0),    # Return to center
        ]
        
        self.current_test = 0
        self.test_start_time = 0.0
        self.max_test_time = 180.0  # 3 minutes per goal
        self.goal_tolerance = 0.5
        
        # Performance metrics
        self.metrics = {
            'total_tests': len(self.test_goals),
            'successful_tests': 0,
            'failed_tests': 0,
            'test_results': [],
            'overall_performance': {},
            'autonomous_features_tested': {
                'path_planning': False,
                'obstacle_avoidance': False,
                'goal_reaching': False,
                'emergency_stop': False,
                'vision_guidance': False,
                'smooth_control': False
            }
        }
        
        # Robot state tracking
        self.current_pose = None
        self.current_goal = None
        self.path_received = False
        self.emergency_active = False
        self.navigation_progress = 0.0
        
        # Performance tracking
        self.path_length = 0.0
        self.distance_traveled = 0.0
        self.collision_count = 0
        self.emergency_stops = 0
        self.min_obstacle_distance = float('inf')
        self.control_smoothness_score = 0.0
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10)
        
        self.emergency_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_callback, 10)
        
        self.progress_sub = self.create_subscription(
            Float64, '/navigation_progress', self.progress_callback, 10)
        
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        
        # Test timer
        self.test_timer = self.create_timer(1.0, self.test_monitor)
        
        # Control smoothness tracking
        self.prev_cmd_vel = None
        self.cmd_vel_history = []
        
        self.get_logger().info("Autonomous Navigation Tester initialized")
        self.get_logger().info(f"Testing {len(self.test_goals)} navigation scenarios")
        
        # Start first test after initialization delay
        self.create_timer(5.0, self.start_next_test)

    def odom_callback(self, msg):
        """Track robot position and calculate metrics"""
        prev_pose = self.current_pose
        self.current_pose = msg.pose.pose
        
        # Calculate distance traveled
        if prev_pose is not None:
            dx = self.current_pose.position.x - prev_pose.position.x
            dy = self.current_pose.position.y - prev_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < 1.0:  # Filter out teleportation
                self.distance_traveled += distance

    def path_callback(self, msg):
        """Monitor path planning functionality"""
        self.path_received = True
        self.metrics['autonomous_features_tested']['path_planning'] = True
        
        # Calculate path length
        if len(msg.poses) > 1:
            path_length = 0.0
            for i in range(1, len(msg.poses)):
                dx = msg.poses[i].pose.position.x - msg.poses[i-1].pose.position.x
                dy = msg.poses[i].pose.position.y - msg.poses[i-1].pose.position.y
                path_length += math.sqrt(dx*dx + dy*dy)
            self.path_length = path_length

    def emergency_callback(self, msg):
        """Monitor emergency stop functionality"""
        if msg.data and not self.emergency_active:
            self.emergency_stops += 1
            self.metrics['autonomous_features_tested']['emergency_stop'] = True
            self.get_logger().warn("Emergency stop activated!")
        
        self.emergency_active = msg.data

    def progress_callback(self, msg):
        """Monitor navigation progress"""
        self.navigation_progress = msg.data

    def laser_callback(self, msg):
        """Monitor obstacle avoidance and safety"""
        if not msg.ranges:
            return
        
        valid_ranges = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        if valid_ranges:
            min_range = min(valid_ranges)
            self.min_obstacle_distance = min(self.min_obstacle_distance, min_range)
            
            # Test obstacle avoidance
            if min_range < 1.0:
                self.metrics['autonomous_features_tested']['obstacle_avoidance'] = True
            
            # Detect collisions
            if min_range < 0.2:
                self.collision_count += 1

    def cmd_vel_callback(self, msg):
        """Monitor control smoothness"""
        if self.prev_cmd_vel is not None:
            # Calculate control smoothness (lower jerk = smoother)
            linear_jerk = abs(msg.linear.x - self.prev_cmd_vel.linear.x)
            angular_jerk = abs(msg.angular.z - self.prev_cmd_vel.angular.z)
            
            self.cmd_vel_history.append({
                'linear_jerk': linear_jerk,
                'angular_jerk': angular_jerk,
                'timestamp': time.time()
            })
            
            # Keep only recent history
            current_time = time.time()
            self.cmd_vel_history = [
                entry for entry in self.cmd_vel_history 
                if current_time - entry['timestamp'] < 10.0
            ]
            
            # Test smooth control
            if linear_jerk < 0.5 and angular_jerk < 0.5:
                self.metrics['autonomous_features_tested']['smooth_control'] = True
        
        self.prev_cmd_vel = msg

    def start_next_test(self):
        """Start the next navigation test"""
        if self.current_test >= len(self.test_goals):
            self.generate_final_report()
            return
        
        # Reset metrics for new test
        self.test_start_time = time.time()
        self.path_received = False
        self.distance_traveled = 0.0
        self.collision_count = 0
        self.min_obstacle_distance = float('inf')
        self.cmd_vel_history = []
        
        # Set new goal
        goal_x, goal_y = self.test_goals[self.current_test]
        self.current_goal = (goal_x, goal_y)
        
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_msg)
        
        self.get_logger().info(f"Test {self.current_test + 1}/{len(self.test_goals)}: "
                              f"Navigating to ({goal_x:.1f}, {goal_y:.1f})")

    def test_monitor(self):
        """Monitor test progress and completion"""
        if self.current_test >= len(self.test_goals):
            return
        
        elapsed_time = time.time() - self.test_start_time
        
        # Check for goal reached
        if self.is_goal_reached():
            self.record_test_result(True, elapsed_time)
            self.current_test += 1
            self.create_timer(3.0, self.start_next_test)  # 3 second delay between tests
            return
        
        # Check for timeout
        if elapsed_time >= self.max_test_time:
            self.record_test_result(False, elapsed_time)
            self.current_test += 1
            self.create_timer(3.0, self.start_next_test)
            return
        
        # Progress update
        if int(elapsed_time) % 30 == 0 and elapsed_time == int(elapsed_time):
            distance_to_goal = self.get_distance_to_goal()
            self.get_logger().info(f"Test progress: {elapsed_time:.0f}s, "
                                  f"distance to goal: {distance_to_goal:.2f}m, "
                                  f"progress: {self.navigation_progress:.1%}")

    def is_goal_reached(self):
        """Check if current goal is reached"""
        if self.current_pose is None or self.current_goal is None:
            return False
        
        distance = self.get_distance_to_goal()
        reached = distance <= self.goal_tolerance
        
        if reached:
            self.metrics['autonomous_features_tested']['goal_reaching'] = True
        
        return reached

    def get_distance_to_goal(self):
        """Calculate distance to current goal"""
        if self.current_pose is None or self.current_goal is None:
            return float('inf')
        
        dx = self.current_pose.position.x - self.current_goal[0]
        dy = self.current_pose.position.y - self.current_goal[1]
        return math.sqrt(dx*dx + dy*dy)

    def record_test_result(self, success, completion_time):
        """Record results of completed test"""
        goal_x, goal_y = self.current_goal
        
        # Calculate efficiency metrics
        straight_line_distance = math.sqrt(goal_x**2 + goal_y**2)  # From origin
        path_efficiency = straight_line_distance / max(self.distance_traveled, 0.001)
        
        # Calculate control smoothness score
        if self.cmd_vel_history:
            avg_linear_jerk = sum(entry['linear_jerk'] for entry in self.cmd_vel_history) / len(self.cmd_vel_history)
            avg_angular_jerk = sum(entry['angular_jerk'] for entry in self.cmd_vel_history) / len(self.cmd_vel_history)
            smoothness_score = 1.0 / (1.0 + avg_linear_jerk + avg_angular_jerk)
        else:
            smoothness_score = 0.0
        
        result = {
            'test_number': self.current_test + 1,
            'goal': (goal_x, goal_y),
            'success': success,
            'completion_time': completion_time,
            'distance_traveled': self.distance_traveled,
            'path_length': self.path_length,
            'path_efficiency': path_efficiency,
            'collision_count': self.collision_count,
            'emergency_stops': self.emergency_stops,
            'min_obstacle_distance': self.min_obstacle_distance,
            'navigation_progress': self.navigation_progress,
            'control_smoothness': smoothness_score,
            'path_planned': self.path_received
        }
        
        self.metrics['test_results'].append(result)
        
        if success:
            self.metrics['successful_tests'] += 1
            status = "SUCCESS"
        else:
            self.metrics['failed_tests'] += 1
            status = "TIMEOUT"
        
        self.get_logger().info(f"Test {self.current_test + 1} completed: {status} "
                              f"in {completion_time:.1f}s, efficiency: {path_efficiency:.2f}")

    def calculate_overall_performance_score(self):
        """Calculate comprehensive performance score (0-100)"""
        if not self.metrics['test_results']:
            return 0.0
        
        # Success rate (40% weight)
        success_rate = self.metrics['successful_tests'] / self.metrics['total_tests']
        success_score = success_rate * 40
        
        # Average efficiency (25% weight)
        successful_tests = [r for r in self.metrics['test_results'] if r['success']]
        if successful_tests:
            avg_efficiency = sum(r['path_efficiency'] for r in successful_tests) / len(successful_tests)
            efficiency_score = min(avg_efficiency, 1.0) * 25
        else:
            efficiency_score = 0
        
        # Safety score (20% weight) - based on collisions and emergency stops
        total_collisions = sum(r['collision_count'] for r in self.metrics['test_results'])
        total_emergencies = sum(r['emergency_stops'] for r in self.metrics['test_results'])
        safety_score = max(0, 20 - total_collisions * 5 - total_emergencies * 2)
        
        # Feature completeness (15% weight)
        features_tested = sum(self.metrics['autonomous_features_tested'].values())
        total_features = len(self.metrics['autonomous_features_tested'])
        feature_score = (features_tested / total_features) * 15
        
        total_score = success_score + efficiency_score + safety_score + feature_score
        return min(100.0, total_score)

    def generate_final_report(self):
        """Generate comprehensive test report"""
        overall_score = self.calculate_overall_performance_score()
        
        self.get_logger().info("=== AUTONOMOUS NAVIGATION SYSTEM TEST RESULTS ===")
        self.get_logger().info(f"Total Tests: {self.metrics['total_tests']}")
        self.get_logger().info(f"Successful: {self.metrics['successful_tests']}")
        self.get_logger().info(f"Failed: {self.metrics['failed_tests']}")
        self.get_logger().info(f"Success Rate: {self.metrics['successful_tests']/self.metrics['total_tests']*100:.1f}%")
        self.get_logger().info("")
        
        # Performance metrics
        if self.metrics['test_results']:
            successful_tests = [r for r in self.metrics['test_results'] if r['success']]
            
            if successful_tests:
                avg_time = sum(r['completion_time'] for r in successful_tests) / len(successful_tests)
                avg_efficiency = sum(r['path_efficiency'] for r in successful_tests) / len(successful_tests)
                avg_smoothness = sum(r['control_smoothness'] for r in successful_tests) / len(successful_tests)
                
                self.get_logger().info("PERFORMANCE METRICS (Successful tests):")
                self.get_logger().info(f"  Average Completion Time: {avg_time:.1f}s")
                self.get_logger().info(f"  Average Path Efficiency: {avg_efficiency:.2f}")
                self.get_logger().info(f"  Average Control Smoothness: {avg_smoothness:.2f}")
        
        # Safety metrics
        total_collisions = sum(r['collision_count'] for r in self.metrics['test_results'])
        total_emergencies = sum(r['emergency_stops'] for r in self.metrics['test_results'])
        
        self.get_logger().info("")
        self.get_logger().info("SAFETY METRICS:")
        self.get_logger().info(f"  Total Collisions: {total_collisions}")
        self.get_logger().info(f"  Emergency Stops: {total_emergencies}")
        
        # Feature testing
        self.get_logger().info("")
        self.get_logger().info("AUTONOMOUS FEATURES TESTED:")
        for feature, tested in self.metrics['autonomous_features_tested'].items():
            status = "✓" if tested else "✗"
            self.get_logger().info(f"  {status} {feature.replace('_', ' ').title()}")
        
        # Overall assessment
        self.get_logger().info("")
        self.get_logger().info(f"OVERALL PERFORMANCE SCORE: {overall_score:.1f}/100")
        
        if overall_score >= 90:
            self.get_logger().info("🏆 EXCELLENT: Autonomous vehicle-grade performance achieved!")
        elif overall_score >= 75:
            self.get_logger().info("✅ GOOD: Professional navigation system performance")
        elif overall_score >= 60:
            self.get_logger().info("⚠️  FAIR: System needs optimization")
        else:
            self.get_logger().info("❌ POOR: System requires significant improvements")
        
        # Save detailed results
        self.save_results_to_file()
        
        self.get_logger().info("=== AUTONOMOUS NAVIGATION TEST COMPLETE ===")
        
        # Shutdown after report
        rclpy.shutdown()

    def save_results_to_file(self):
        """Save detailed test results to JSON file"""
        # Create results directory
        os.makedirs('test_results', exist_ok=True)
        
        # Prepare results data
        results_data = {
            'test_timestamp': datetime.now().isoformat(),
            'test_configuration': {
                'test_goals': self.test_goals,
                'max_test_time': self.max_test_time,
                'goal_tolerance': self.goal_tolerance
            },
            'metrics': self.metrics,
            'overall_score': self.calculate_overall_performance_score()
        }
        
        # Save to file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'test_results/autonomous_navigation_test_{timestamp}.json'
        
        with open(filename, 'w') as f:
            json.dump(results_data, f, indent=2)
        
        self.get_logger().info(f"Detailed results saved to: {filename}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = AutonomousNavigationTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
