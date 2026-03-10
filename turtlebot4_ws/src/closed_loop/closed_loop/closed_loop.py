#!/usr/bin/env python3
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class LaserCloseLoop(Node):
    def __init__(self):
        super().__init__('laser_closeloop')
        self.debug = True
        # Subscriber to the laser scan topic
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # Publisher for velocity commands
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.front_value_list = []
        self.motion_move = Twist()
        self.motion_stop = Twist()
        # Set constant speed to move forward
        self.motion_move.linear.x = 0.15
        # Set speed to stop
        self.motion_stop.linear.x = 0.0
        self.desired_distance = 2.0  # Distance to travel in meters
        
    def scan_callback(self, msg):
        # The index of the front value might need to be adjusted based on your sensor
        ranges = msg.ranges
        n_ranges = len(ranges)
        current_front_value = ranges[int(n_ranges * 0.25)]
        self.front_value_list.append(current_front_value)
        print(self.front_value_list)
        
        if len(self.front_value_list) > 2:
            # Adjust condition based on desired stopping distance
            if self.front_value_list[-1] < (self.front_value_list[0] - self.desired_distance):
                self.pub.publish(self.motion_stop)
                self.get_logger().info("Reached goal, stopping...")
                rclpy.shutdown()
            else:
                self.pub.publish(self.motion_move)
                if self.debug:
                    self.get_logger().info(f"Value ahead: {current_front_value}")
                    self.get_logger().info(f"Distance traveled: {self.front_value_list[0] - self.front_value_list[-1]}")

def main(args=None):
    rclpy.init(args=args)
    scan_cl = LaserCloseLoop()
    rclpy.spin(scan_cl)
    scan_cl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()