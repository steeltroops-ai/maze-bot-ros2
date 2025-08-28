#!/usr/bin/env python3

"""
Navigation Algorithm Comparison Test
Compares performance of different navigation algorithms
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import json
import os

class NavigationAlgorithmTester(Node):
    def __init__(self):
        super().__init__('navigation_algorithm_tester')
        
        # Test configuration
        self.test_runs = 5
        self.max_test_time = 300.0  # 5 minutes per run
        self.goal_tolerance = 0.5
        
        # Goal positions to test
        self.test_goals = [
            (4.0, 4.0),    # Standard goal
            (3.0, -3.0),   # Different quadrant
            (-2.0, 3.0),   # Across maze
            (0.0, 4.5),    # Edge goal
        ]
        
        # Current test state
        self.current_run = 0
        self.current_goal_idx = 0
        self.test_start_time = 0.0
        self.run_results = []
        
        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.path_length = 0.0
        self.previous_x = 0.0
        self.previous_y = 0.0
        
        # Performance metrics
        self.collision_count = 0
        self.stuck_time = 0.0
        self.last_progress_time = 0.0
        self.min_obstacle_distance = float('inf')
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        
        # Test timer
        self.test_timer = self.create_timer(1.0, self.test_monitor)
        
        self.get_logger().info("Navigation Algorithm Tester initialized")
        self.get_logger().info("Testing {} goals with {} runs each".format(
            len(self.test_goals), self.test_runs))
        
        self.start_next_test()

    def odom_callback(self, msg):
        """Track robot position and calculate metrics"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Calculate path length
        if self.previous_x != 0.0 or self.previous_y != 0.0:
            dx = self.current_x - self.previous_x
            dy = self.current_y - self.previous_y
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < 1.0:  # Filter out teleportation
                self.path_length += distance
        
        self.previous_x = self.current_x
        self.previous_y = self.current_y
        
        # Check for progress (movement)
        if self.has_moved_significantly():
            self.last_progress_time = time.time()

    def laser_callback(self, msg):
        """Monitor for collisions and near-misses"""
        if not msg.ranges:
            return
        
        min_range = min([r for r in msg.ranges if r > 0.0 and math.isfinite(r)])
        self.min_obstacle_distance = min(self.min_obstacle_distance, min_range)
        
        # Detect potential collisions
        if min_range < 0.2:
            self.collision_count += 1

    def has_moved_significantly(self):
        """Check if robot has moved significantly from start position"""
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.sqrt(dx*dx + dy*dy) > 0.1

    def is_goal_reached(self):
        """Check if current goal is reached"""
        if self.current_goal_idx >= len(self.test_goals):
            return False
        
        goal_x, goal_y = self.test_goals[self.current_goal_idx]
        dx = self.current_x - goal_x
        dy = self.current_y - goal_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance <= self.goal_tolerance

    def start_next_test(self):
        """Initialize next test run"""
        if self.current_goal_idx >= len(self.test_goals):
            self.current_goal_idx = 0
            self.current_run += 1
        
        if self.current_run >= self.test_runs:
            self.generate_final_report()
            rclpy.shutdown()
            return
        
        # Reset metrics for new test
        self.test_start_time = time.time()
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.path_length = 0.0
        self.collision_count = 0
        self.stuck_time = 0.0
        self.last_progress_time = time.time()
        self.min_obstacle_distance = float('inf')
        
        goal_x, goal_y = self.test_goals[self.current_goal_idx]
        self.get_logger().info("Starting test run {}/{} to goal ({:.1f}, {:.1f})".format(
            self.current_run + 1, self.test_runs, goal_x, goal_y))

    def test_monitor(self):
        """Monitor test progress and completion"""
        if self.test_start_time == 0.0:
            return
        
        elapsed_time = time.time() - self.test_start_time
        
        # Check for goal reached
        if self.is_goal_reached():
            self.record_test_result(True, elapsed_time)
            self.current_goal_idx += 1
            self.start_next_test()
            return
        
        # Check for timeout
        if elapsed_time >= self.max_test_time:
            self.record_test_result(False, elapsed_time)
            self.current_goal_idx += 1
            self.start_next_test()
            return
        
        # Check for stuck condition
        time_since_progress = time.time() - self.last_progress_time
        if time_since_progress > 30.0:  # Stuck for 30 seconds
            self.stuck_time += 1.0
        
        # Progress update
        if int(elapsed_time) % 30 == 0 and elapsed_time == int(elapsed_time):
            goal_x, goal_y = self.test_goals[self.current_goal_idx]
            distance_to_goal = math.sqrt(
                (self.current_x - goal_x)**2 + (self.current_y - goal_y)**2)
            self.get_logger().info("Test progress: {:.0f}s, distance to goal: {:.2f}m".format(
                elapsed_time, distance_to_goal))

    def record_test_result(self, success, completion_time):
        """Record results of completed test"""
        goal_x, goal_y = self.test_goals[self.current_goal_idx]
        
        # Calculate efficiency metrics
        straight_line_distance = math.sqrt(
            (goal_x - self.start_x)**2 + (goal_y - self.start_y)**2)
        
        path_efficiency = straight_line_distance / max(self.path_length, 0.001)
        average_speed = self.path_length / max(completion_time, 0.001)
        
        result = {
            'run': self.current_run + 1,
            'goal': (goal_x, goal_y),
            'success': success,
            'completion_time': completion_time,
            'path_length': self.path_length,
            'path_efficiency': path_efficiency,
            'average_speed': average_speed,
            'collision_count': self.collision_count,
            'stuck_time': self.stuck_time,
            'min_obstacle_distance': self.min_obstacle_distance,
            'start_position': (self.start_x, self.start_y),
            'final_position': (self.current_x, self.current_y)
        }
        
        self.run_results.append(result)
        
        status = "SUCCESS" if success else "TIMEOUT"
        self.get_logger().info("Test completed: {} in {:.1f}s, path: {:.2f}m, efficiency: {:.2f}".format(
            status, completion_time, self.path_length, path_efficiency))

    def calculate_algorithm_score(self):
        """Calculate overall algorithm performance score"""
        if not self.run_results:
            return 0.0
        
        successful_runs = [r for r in self.run_results if r['success']]
        success_rate = len(successful_runs) / len(self.run_results)
        
        if not successful_runs:
            return 0.0
        
        # Calculate average metrics for successful runs
        avg_completion_time = sum(r['completion_time'] for r in successful_runs) / len(successful_runs)
        avg_path_efficiency = sum(r['path_efficiency'] for r in successful_runs) / len(successful_runs)
        avg_collisions = sum(r['collision_count'] for r in successful_runs) / len(successful_runs)
        avg_stuck_time = sum(r['stuck_time'] for r in successful_runs) / len(successful_runs)
        
        # Score calculation (0-100)
        score = 100.0
        score *= success_rate  # Success rate multiplier
        score *= min(1.0, 180.0 / max(avg_completion_time, 30.0))  # Time efficiency
        score *= min(1.0, avg_path_efficiency)  # Path efficiency
        score -= min(avg_collisions * 5, 20)  # Collision penalty
        score -= min(avg_stuck_time * 0.1, 10)  # Stuck time penalty
        
        return max(0.0, score)

    def generate_final_report(self):
        """Generate comprehensive test report"""
        self.get_logger().info("=== NAVIGATION ALGORITHM TEST RESULTS ===")
        
        total_tests = len(self.run_results)
        successful_tests = len([r for r in self.run_results if r['success']])
        success_rate = successful_tests / total_tests if total_tests > 0 else 0.0
        
        self.get_logger().info("OVERALL PERFORMANCE:")
        self.get_logger().info("  Total Tests: {}".format(total_tests))
        self.get_logger().info("  Successful: {}".format(successful_tests))
        self.get_logger().info("  Success Rate: {:.1f}%".format(success_rate * 100))
        
        if successful_tests > 0:
            successful_runs = [r for r in self.run_results if r['success']]
            
            avg_time = sum(r['completion_time'] for r in successful_runs) / len(successful_runs)
            avg_path_length = sum(r['path_length'] for r in successful_runs) / len(successful_runs)
            avg_efficiency = sum(r['path_efficiency'] for r in successful_runs) / len(successful_runs)
            avg_speed = sum(r['average_speed'] for r in successful_runs) / len(successful_runs)
            total_collisions = sum(r['collision_count'] for r in successful_runs)
            
            self.get_logger().info("")
            self.get_logger().info("PERFORMANCE METRICS (Successful runs):")
            self.get_logger().info("  Average Completion Time: {:.1f}s".format(avg_time))
            self.get_logger().info("  Average Path Length: {:.2f}m".format(avg_path_length))
            self.get_logger().info("  Average Path Efficiency: {:.2f}".format(avg_efficiency))
            self.get_logger().info("  Average Speed: {:.2f}m/s".format(avg_speed))
            self.get_logger().info("  Total Collisions: {}".format(total_collisions))
        
        # Algorithm score
        algorithm_score = self.calculate_algorithm_score()
        self.get_logger().info("")
        self.get_logger().info("ALGORITHM PERFORMANCE SCORE: {:.1f}/100".format(algorithm_score))
        
        if algorithm_score >= 85:
            self.get_logger().info("✓ EXCELLENT: Algorithm performs exceptionally well")
        elif algorithm_score >= 70:
            self.get_logger().info("✓ GOOD: Algorithm performance is satisfactory")
        elif algorithm_score >= 50:
            self.get_logger().info("⚠ FAIR: Algorithm needs optimization")
        else:
            self.get_logger().info("✗ POOR: Algorithm requires significant improvement")
        
        # Save detailed results to file
        self.save_results_to_file()
        
        self.get_logger().info("=== NAVIGATION TEST COMPLETE ===")

    def save_results_to_file(self):
        """Save detailed test results to JSON file"""
        results_data = {
            'test_configuration': {
                'test_runs': self.test_runs,
                'max_test_time': self.max_test_time,
                'goal_tolerance': self.goal_tolerance,
                'test_goals': self.test_goals
            },
            'results': self.run_results,
            'summary': {
                'total_tests': len(self.run_results),
                'successful_tests': len([r for r in self.run_results if r['success']]),
                'success_rate': len([r for r in self.run_results if r['success']]) / len(self.run_results),
                'algorithm_score': self.calculate_algorithm_score()
            }
        }
        
        # Create results directory if it doesn't exist
        os.makedirs('test_results', exist_ok=True)
        
        # Save to file with timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f'test_results/navigation_test_{timestamp}.json'
        
        with open(filename, 'w') as f:
            json.dump(results_data, f, indent=2)
        
        self.get_logger().info("Detailed results saved to: {}".format(filename))

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = NavigationAlgorithmTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
