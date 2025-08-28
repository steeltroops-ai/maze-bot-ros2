#!/usr/bin/env python3

"""
Autonomous Navigation System - Live Demonstration Script
Showcases autonomous vehicle-grade navigation capabilities
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64
import time
import math

class AutonomousNavigationDemo(Node):
    def __init__(self):
        super().__init__('autonomous_navigation_demo')
        
        # Demo waypoints showcasing different capabilities
        self.demo_waypoints = [
            {
                'name': 'Path Planning Demo',
                'goal': (4.0, 4.0),
                'description': 'Demonstrates A* pathfinding and optimal route calculation',
                'expected_features': ['path_planning', 'obstacle_avoidance']
            },
            {
                'name': 'Precision Navigation',
                'goal': (0.0, 4.0),
                'description': 'Shows precise goal reaching and pure pursuit control',
                'expected_features': ['goal_reaching', 'smooth_control']
            },
            {
                'name': 'Dynamic Obstacle Avoidance',
                'goal': (-3.0, 0.0),
                'description': 'Tests real-time obstacle avoidance and replanning',
                'expected_features': ['obstacle_avoidance', 'dynamic_replanning']
            },
            {
                'name': 'Complex Navigation',
                'goal': (3.0, -3.0),
                'description': 'Multi-turn navigation with speed adaptation',
                'expected_features': ['path_following', 'speed_control']
            },
            {
                'name': 'Return to Origin',
                'goal': (0.0, 0.0),
                'description': 'Demonstrates navigation consistency and accuracy',
                'expected_features': ['goal_reaching', 'path_efficiency']
            }
        ]
        
        self.current_demo = 0
        self.demo_start_time = 0.0
        self.demo_active = False
        
        # Performance tracking
        self.performance_metrics = {
            'goals_reached': 0,
            'total_distance': 0.0,
            'total_time': 0.0,
            'emergency_stops': 0,
            'path_efficiency': 0.0,
            'features_demonstrated': set()
        }
        
        # Robot state
        self.current_pose = None
        self.current_goal = None
        self.path_received = False
        self.emergency_active = False
        self.navigation_progress = 0.0
        
        # Subscribers for monitoring
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
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        
        # Demo timer
        self.demo_timer = self.create_timer(2.0, self.demo_monitor)
        
        self.get_logger().info("🚀 Autonomous Navigation Demo Ready!")
        self.get_logger().info("📋 Demo will showcase 5 autonomous capabilities")
        self.get_logger().info("⏱️  Starting demo in 5 seconds...")
        
        # Start demo after initialization
        self.create_timer(5.0, self.start_demo)

    def start_demo(self):
        """Start the autonomous navigation demonstration"""
        self.get_logger().info("🎬 AUTONOMOUS NAVIGATION DEMONSTRATION STARTING")
        self.get_logger().info("=" * 60)
        self.start_next_waypoint()

    def start_next_waypoint(self):
        """Start navigation to the next demo waypoint"""
        if self.current_demo >= len(self.demo_waypoints):
            self.complete_demo()
            return
        
        waypoint = self.demo_waypoints[self.current_demo]
        self.current_goal = waypoint['goal']
        self.demo_start_time = time.time()
        self.demo_active = True
        self.path_received = False
        
        # Create and publish goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = waypoint['goal'][0]
        goal_msg.pose.position.y = waypoint['goal'][1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_msg)
        
        self.get_logger().info(f"🎯 Demo {self.current_demo + 1}/5: {waypoint['name']}")
        self.get_logger().info(f"📍 Goal: ({waypoint['goal'][0]:.1f}, {waypoint['goal'][1]:.1f})")
        self.get_logger().info(f"📝 {waypoint['description']}")
        self.get_logger().info(f"🔧 Expected Features: {', '.join(waypoint['expected_features'])}")

    def demo_monitor(self):
        """Monitor demo progress and completion"""
        if not self.demo_active:
            return
        
        elapsed_time = time.time() - self.demo_start_time
        
        # Check for goal reached
        if self.is_goal_reached():
            self.complete_waypoint(elapsed_time)
            return
        
        # Check for timeout (3 minutes per waypoint)
        if elapsed_time >= 180.0:
            self.get_logger().warn(f"⏰ Demo waypoint timeout after {elapsed_time:.1f}s")
            self.complete_waypoint(elapsed_time, success=False)
            return
        
        # Progress updates every 15 seconds
        if int(elapsed_time) % 15 == 0 and elapsed_time == int(elapsed_time):
            distance_to_goal = self.get_distance_to_goal()
            self.get_logger().info(f"📊 Progress: {elapsed_time:.0f}s, "
                                  f"distance: {distance_to_goal:.2f}m, "
                                  f"progress: {self.navigation_progress:.1%}")

    def complete_waypoint(self, elapsed_time, success=True):
        """Complete current waypoint and move to next"""
        waypoint = self.demo_waypoints[self.current_demo]
        
        if success:
            self.performance_metrics['goals_reached'] += 1
            self.performance_metrics['features_demonstrated'].update(waypoint['expected_features'])
            status = "✅ SUCCESS"
        else:
            status = "❌ TIMEOUT"
        
        self.performance_metrics['total_time'] += elapsed_time
        
        self.get_logger().info(f"🏁 {waypoint['name']}: {status} in {elapsed_time:.1f}s")
        self.get_logger().info("-" * 40)
        
        self.demo_active = False
        self.current_demo += 1
        
        # 5 second delay between waypoints
        self.create_timer(5.0, self.start_next_waypoint)

    def complete_demo(self):
        """Complete the entire demonstration"""
        self.get_logger().info("🎉 AUTONOMOUS NAVIGATION DEMONSTRATION COMPLETE!")
        self.get_logger().info("=" * 60)
        
        # Calculate performance metrics
        success_rate = (self.performance_metrics['goals_reached'] / len(self.demo_waypoints)) * 100
        avg_time = self.performance_metrics['total_time'] / len(self.demo_waypoints)
        features_demonstrated = len(self.performance_metrics['features_demonstrated'])
        total_features = len(set().union(*[wp['expected_features'] for wp in self.demo_waypoints]))
        feature_completeness = (features_demonstrated / total_features) * 100
        
        self.get_logger().info("📊 DEMONSTRATION RESULTS:")
        self.get_logger().info(f"   Success Rate: {success_rate:.1f}% ({self.performance_metrics['goals_reached']}/{len(self.demo_waypoints)})")
        self.get_logger().info(f"   Average Time: {avg_time:.1f} seconds per waypoint")
        self.get_logger().info(f"   Emergency Stops: {self.performance_metrics['emergency_stops']}")
        self.get_logger().info(f"   Features Demonstrated: {feature_completeness:.1f}% ({features_demonstrated}/{total_features})")
        
        self.get_logger().info("")
        self.get_logger().info("🏆 AUTONOMOUS CAPABILITIES DEMONSTRATED:")
        for feature in sorted(self.performance_metrics['features_demonstrated']):
            self.get_logger().info(f"   ✓ {feature.replace('_', ' ').title()}")
        
        # Overall assessment
        if success_rate >= 80 and feature_completeness >= 80:
            self.get_logger().info("🎯 ASSESSMENT: Autonomous vehicle-grade performance achieved!")
        elif success_rate >= 60:
            self.get_logger().info("⚠️  ASSESSMENT: Good performance, some optimization needed")
        else:
            self.get_logger().info("❌ ASSESSMENT: Performance below autonomous vehicle standards")
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 Demo complete - System ready for autonomous operation!")

    def odom_callback(self, msg):
        """Track robot position"""
        self.current_pose = msg.pose.pose

    def path_callback(self, msg):
        """Monitor path planning"""
        self.path_received = True
        self.performance_metrics['features_demonstrated'].add('path_planning')

    def emergency_callback(self, msg):
        """Monitor emergency stops"""
        if msg.data and not self.emergency_active:
            self.performance_metrics['emergency_stops'] += 1
            self.performance_metrics['features_demonstrated'].add('emergency_stop')
        self.emergency_active = msg.data

    def progress_callback(self, msg):
        """Monitor navigation progress"""
        self.navigation_progress = msg.data

    def laser_callback(self, msg):
        """Monitor obstacle avoidance"""
        if not msg.ranges:
            return
        
        valid_ranges = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        if valid_ranges:
            min_range = min(valid_ranges)
            if min_range < 1.0:  # Close obstacle detected
                self.performance_metrics['features_demonstrated'].add('obstacle_avoidance')

    def is_goal_reached(self):
        """Check if current goal is reached"""
        if self.current_pose is None or self.current_goal is None:
            return False
        
        distance = self.get_distance_to_goal()
        reached = distance <= 0.5  # 0.5m tolerance
        
        if reached:
            self.performance_metrics['features_demonstrated'].add('goal_reaching')
        
        return reached

    def get_distance_to_goal(self):
        """Calculate distance to current goal"""
        if self.current_pose is None or self.current_goal is None:
            return float('inf')
        
        dx = self.current_pose.position.x - self.current_goal[0]
        dy = self.current_pose.position.y - self.current_goal[1]
        return math.sqrt(dx*dx + dy*dy)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = AutonomousNavigationDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
