#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import signal
import time
import sys

class CircleDriver(Node):
    def __init__(self):
        super().__init__('circle_driver')
        
        # Create publisher for velocity commands
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize Twist message
        self.cmd = Twist()
        self.cmd.linear.x = 0.1   # Linear velocity: 0.1 m/s forward
        self.cmd.angular.z = 0.1  # Angular velocity: 0.1 rad/s (counterclockwise)
        
        # Calculate radius of the circle
        radius = self.cmd.linear.x / abs(self.cmd.angular.z)
        
        # Create timer to publish commands at 10Hz
        self.timer = self.create_timer(0.1, lambda: self.pub.publish(self.cmd))
        
        # Log circle parameters
        self.get_logger().info(
            f"Driving in a circle: v={self.cmd.linear.x:.2f}m/s, "
            f"ω={self.cmd.angular.z:.2f}rad/s, r={radius:.2f}m"
        )
    
    def stop_robot(self):
        """Stop the robot by publishing zero velocities"""
        self.pub.publish(Twist())  # All values default to 0
        time.sleep(0.1)
        self.get_logger().info("Robot stopped")

def main(args=None):
    rclpy.init(args=args)
    node = CircleDriver()
    
    # Handle Ctrl-C gracefully
    def custom_sigint(signum, frame):
        node.get_logger().info("Ctrl-C detected: stopping robot")
        node.stop_robot()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, custom_sigint)
    
    # Keep node running
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()

if __name__ == "__main__":
    main()