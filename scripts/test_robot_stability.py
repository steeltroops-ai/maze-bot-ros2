#!/usr/bin/env python3

"""
Robot Stability Test Script
Tests robot physics stability and detects flipping/tipping events
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time
import numpy as np

class RobotStabilityTester(Node):
    def __init__(self):
        super().__init__('robot_stability_tester')
        
        # Test parameters
        self.test_duration = 120.0  # Test for 2 minutes
        self.start_time = time.time()
        
        # Stability metrics
        self.max_roll = 0.0
        self.max_pitch = 0.0
        self.max_angular_velocity = 0.0
        self.flip_events = 0
        self.instability_events = 0
        self.velocity_commands_sent = 0
        
        # Stability thresholds
        self.flip_threshold = math.pi / 3  # 60 degrees
        self.instability_threshold = math.pi / 6  # 30 degrees
        self.max_safe_angular_vel = 5.0  # rad/s
        
        # Data storage
        self.orientation_history = []
        self.velocity_history = []
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # Publisher for test commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer for test execution
        self.test_timer = self.create_timer(0.1, self.stability_test_loop)
        self.command_timer = self.create_timer(2.0, self.send_test_commands)
        
        self.get_logger().info("Robot Stability Tester started - testing for {} seconds".format(self.test_duration))

    def odom_callback(self, msg):
        """Monitor robot orientation and detect instability"""
        # Extract orientation (quaternion to euler)
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to roll, pitch, yaw
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w)
        
        # Update maximum values
        self.max_roll = max(self.max_roll, abs(roll))
        self.max_pitch = max(self.max_pitch, abs(pitch))
        
        # Store orientation history
        self.orientation_history.append({
            'time': time.time(),
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        })
        
        # Keep only recent history (last 10 seconds)
        current_time = time.time()
        self.orientation_history = [
            entry for entry in self.orientation_history 
            if current_time - entry['time'] < 10.0
        ]
        
        # Check for flip events
        if abs(roll) > self.flip_threshold or abs(pitch) > self.flip_threshold:
            if self.flip_events == 0:  # First flip event
                self.get_logger().error("ROBOT FLIP DETECTED! Roll: {:.2f}°, Pitch: {:.2f}°".format(
                    math.degrees(roll), math.degrees(pitch)))
            self.flip_events += 1
        
        # Check for instability
        elif abs(roll) > self.instability_threshold or abs(pitch) > self.instability_threshold:
            self.instability_events += 1
            if self.instability_events % 50 == 0:  # Log every 5 seconds
                self.get_logger().warn("Robot instability detected. Roll: {:.2f}°, Pitch: {:.2f}°".format(
                    math.degrees(roll), math.degrees(pitch)))
        
        # Monitor angular velocity
        angular_vel = msg.twist.twist.angular
        total_angular_vel = math.sqrt(angular_vel.x**2 + angular_vel.y**2 + angular_vel.z**2)
        self.max_angular_velocity = max(self.max_angular_velocity, total_angular_vel)
        
        if total_angular_vel > self.max_safe_angular_vel:
            self.get_logger().warn("High angular velocity detected: {:.2f} rad/s".format(total_angular_vel))

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    def send_test_commands(self):
        """Send various test commands to stress-test robot stability"""
        if time.time() - self.start_time > self.test_duration:
            return
        
        cmd_vel = Twist()
        test_phase = int((time.time() - self.start_time) / 10) % 6
        
        if test_phase == 0:
            # Forward motion
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = 0.0
        elif test_phase == 1:
            # Turning motion
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 1.0
        elif test_phase == 2:
            # Sharp turn
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 1.5
        elif test_phase == 3:
            # Backward motion
            cmd_vel.linear.x = -0.3
            cmd_vel.angular.z = 0.0
        elif test_phase == 4:
            # Figure-8 pattern
            cmd_vel.linear.x = 0.4
            cmd_vel.angular.z = 0.8 * math.sin(time.time() - self.start_time)
        else:
            # Stop
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd_vel)
        self.velocity_commands_sent += 1

    def stability_test_loop(self):
        """Main test monitoring loop"""
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time >= self.test_duration:
            self.report_results()
            rclpy.shutdown()
        else:
            remaining_time = self.test_duration - elapsed_time
            if int(remaining_time) % 10 == 0 and remaining_time == int(remaining_time):
                self.get_logger().info("Stability test progress: {:.0f}s remaining".format(remaining_time))

    def calculate_stability_score(self):
        """Calculate overall stability score (0-100)"""
        score = 100.0
        
        # Penalize flip events heavily
        score -= self.flip_events * 50
        
        # Penalize instability events
        score -= min(self.instability_events * 0.1, 30)
        
        # Penalize excessive roll/pitch
        if self.max_roll > self.instability_threshold:
            score -= min((self.max_roll - self.instability_threshold) * 100, 20)
        if self.max_pitch > self.instability_threshold:
            score -= min((self.max_pitch - self.instability_threshold) * 100, 20)
        
        # Penalize excessive angular velocity
        if self.max_angular_velocity > self.max_safe_angular_vel:
            score -= min((self.max_angular_velocity - self.max_safe_angular_vel) * 5, 10)
        
        return max(0.0, score)

    def report_results(self):
        """Generate comprehensive stability test report"""
        stability_score = self.calculate_stability_score()
        
        self.get_logger().info("=== ROBOT STABILITY TEST RESULTS ===")
        self.get_logger().info("Test Duration: {:.1f} seconds".format(self.test_duration))
        self.get_logger().info("Commands Sent: {}".format(self.velocity_commands_sent))
        self.get_logger().info("")
        
        self.get_logger().info("STABILITY METRICS:")
        self.get_logger().info("  Maximum Roll: {:.2f}° ({:.3f} rad)".format(
            math.degrees(self.max_roll), self.max_roll))
        self.get_logger().info("  Maximum Pitch: {:.2f}° ({:.3f} rad)".format(
            math.degrees(self.max_pitch), self.max_pitch))
        self.get_logger().info("  Maximum Angular Velocity: {:.2f} rad/s".format(self.max_angular_velocity))
        self.get_logger().info("")
        
        self.get_logger().info("STABILITY EVENTS:")
        self.get_logger().info("  Flip Events: {}".format(self.flip_events))
        self.get_logger().info("  Instability Events: {}".format(self.instability_events))
        self.get_logger().info("")
        
        # Overall assessment
        self.get_logger().info("OVERALL STABILITY SCORE: {:.1f}/100".format(stability_score))
        
        if stability_score >= 90:
            self.get_logger().info("✓ EXCELLENT: Robot demonstrates excellent stability")
        elif stability_score >= 75:
            self.get_logger().info("✓ GOOD: Robot stability is acceptable for navigation")
        elif stability_score >= 50:
            self.get_logger().info("⚠ FAIR: Robot stability needs improvement")
        else:
            self.get_logger().info("✗ POOR: Robot stability is inadequate - requires physics tuning")
        
        # Recommendations
        if self.flip_events > 0:
            self.get_logger().info("RECOMMENDATION: Lower center of mass, increase base width")
        if self.max_roll > self.instability_threshold or self.max_pitch > self.instability_threshold:
            self.get_logger().info("RECOMMENDATION: Adjust inertial properties and contact parameters")
        if self.max_angular_velocity > self.max_safe_angular_vel:
            self.get_logger().info("RECOMMENDATION: Reduce maximum angular velocity limits")
        
        self.get_logger().info("=== STABILITY TEST COMPLETE ===")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = RobotStabilityTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
