#!/usr/bin/env python3

"""
Navigation Test Script for Maze Bot
This script provides automated testing for the navigation system
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class NavigationTester(Node):
    def __init__(self):
        super().__init__('navigation_tester')
        
        # Test parameters
        self.test_duration = 60.0  # Test for 60 seconds
        self.start_time = time.time()
        
        # Test metrics
        self.collision_detected = False
        self.min_obstacle_distance = float('inf')
        self.max_speed_recorded = 0.0
        self.distance_traveled = 0.0
        self.previous_position = None
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # Timer for test monitoring
        self.timer = self.create_timer(1.0, self.test_monitor)
        
        self.get_logger().info("Navigation Tester started - monitoring for {} seconds".format(self.test_duration))

    def laser_callback(self, msg):
        """Monitor laser scan for collision detection"""
        if msg.ranges:
            min_range = min([r for r in msg.ranges if r > 0.0])
            self.min_obstacle_distance = min(self.min_obstacle_distance, min_range)
            
            # Check for potential collision (very close obstacle)
            if min_range < 0.2:
                if not self.collision_detected:
                    self.get_logger().warn("Potential collision detected! Min distance: {:.2f}m".format(min_range))
                    self.collision_detected = True

    def cmd_vel_callback(self, msg):
        """Monitor velocity commands"""
        linear_speed = abs(msg.linear.x)
        angular_speed = abs(msg.angular.z)
        total_speed = math.sqrt(linear_speed**2 + angular_speed**2)
        
        self.max_speed_recorded = max(self.max_speed_recorded, total_speed)

    def odom_callback(self, msg):
        """Monitor odometry for distance calculation"""
        current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        if self.previous_position is not None:
            dx = current_pos[0] - self.previous_position[0]
            dy = current_pos[1] - self.previous_position[1]
            distance_increment = math.sqrt(dx**2 + dy**2)
            self.distance_traveled += distance_increment
        
        self.previous_position = current_pos

    def test_monitor(self):
        """Monitor test progress and report results"""
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time >= self.test_duration:
            self.report_results()
            rclpy.shutdown()
        else:
            remaining_time = self.test_duration - elapsed_time
            self.get_logger().info("Test progress: {:.1f}s remaining".format(remaining_time))

    def report_results(self):
        """Generate test report"""
        self.get_logger().info("=== NAVIGATION TEST RESULTS ===")
        self.get_logger().info("Test Duration: {:.1f} seconds".format(self.test_duration))
        self.get_logger().info("Distance Traveled: {:.2f} meters".format(self.distance_traveled))
        self.get_logger().info("Average Speed: {:.2f} m/s".format(self.distance_traveled / self.test_duration))
        self.get_logger().info("Max Speed Recorded: {:.2f} m/s".format(self.max_speed_recorded))
        self.get_logger().info("Minimum Obstacle Distance: {:.2f} meters".format(self.min_obstacle_distance))
        
        # Safety assessment
        if self.collision_detected:
            self.get_logger().warn("⚠ SAFETY CONCERN: Potential collision detected!")
        else:
            self.get_logger().info("✓ SAFETY: No collisions detected")
        
        if self.min_obstacle_distance > 0.3:
            self.get_logger().info("✓ OBSTACLE AVOIDANCE: Maintained safe distance")
        else:
            self.get_logger().warn("⚠ OBSTACLE AVOIDANCE: Got very close to obstacles")
        
        if self.distance_traveled > 1.0:
            self.get_logger().info("✓ MOBILITY: Robot is moving effectively")
        else:
            self.get_logger().warn("⚠ MOBILITY: Limited movement detected")
        
        self.get_logger().info("=== TEST COMPLETE ===")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = NavigationTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
