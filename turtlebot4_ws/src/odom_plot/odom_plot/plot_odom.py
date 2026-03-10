#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription
        
        # Lists to store the x and y data
        self.x_data = []
        self.y_data = []

    def odom_callback(self, msg):
        # Extract the x and y positions
        position = msg.pose.pose.position
        x = position.x
        y = position.y
        
        # Log the position
        self.get_logger().info(f"Position -> x: {x}, y: {y}")
        
        # Append the x and y positions to the lists
        self.x_data.append(x)
        self.y_data.append(y)

    def plot_data(self):
        # Plot x and y data
        plt.figure()
        plt.plot(self.x_data, self.y_data, 'b-', linewidth=2, label='Odometry Path')  # Added line style
        plt.title('Odometry X-Y Path')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')  # Equal aspect ratio
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    odom_plotter = OdomPlotter()
    
    try:
        rclpy.spin(odom_plotter)
    except KeyboardInterrupt:
        pass
    finally:
        # On shutdown, plot the data
        odom_plotter.get_logger().info('Shutting down, plotting data...')
        odom_plotter.plot_data()
        odom_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()