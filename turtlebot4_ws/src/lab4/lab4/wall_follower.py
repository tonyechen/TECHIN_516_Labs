import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import String


class PIDController():
    def __init__(
        self, 
        kp: float, 
        ki: float, 
        kd: float, 
        setpoint: float,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._prev_error = 0.0
        self._integral = 0.0
        self.debug = False
    
    def __call__(
        self, 
        system_measurement: float,
        dt: float
    ) -> float:
        """ compute PID output """
        assert dt > 0, "time must be positive"
        error = self.setpoint - system_measurement
        if self.debug:
            print(f"PID Error: {error}")
        p_out = self.kp * error
        self._integral += error * dt
        i_out = self.ki * self._integral
        derivative = (error - self._prev_error) / dt 
        d_out = self.kd * derivative
        self._prev_error = error
        out = p_out + i_out + d_out
        if self.debug:
            print(f"PID P: {p_out}, I: {i_out}, D: {d_out}, OUT: {out}")
        return out


class WallFollowPIDNode(Node):
    def __init__(self):
        super().__init__('wall_follow_pid_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.last_scan = None
        kp = 10.0
        ki = 0.01
        kd = 5.5
        
        self.pid = PIDController(kp=kp, ki=ki, kd=kd, setpoint=0.4)
        self.prev_time = self.get_clock().now()
        self.debug = False
        self.run = False

        # subscribe to orchestrator state
        self.create_subscription(
            String,
            '/orchestrator/state',
            self.orchestrator_state_callback,
            10
        )

        # stop when aruco marker is detected
        self.create_subscription(
            PoseArray,
            '/aruco_detections/poses',
            self._aruco_cb,
            10
        )

    def orchestrator_state_callback(self, msg: String):
        if msg.data == 'EXPLORING':
            self.run = True
        else:
            self.run = False

    def _aruco_cb(self, msg: PoseArray):
        if msg.poses and self.run:
            
            if self.run:
                self.cmd_pub.publish(Twist())
            self.run = False
            self.get_logger().info('Aruco marker detected — wall follower stopped')

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def control_loop(self):
        if self.run == False:
            return
        if self.last_scan is None:
            return
        scan_ranges = np.array(self.last_scan.ranges)
        scan_ranges[np.isinf(scan_ranges)] = self.last_scan.range_max
        n_ranges = len(scan_ranges)
        angle_increment = self.last_scan.angle_increment
        front_index = int(n_ranges * 0.25)
        front_indices = np.arange(int(front_index - (0.3 / angle_increment)),
                                  int(front_index + (0.3 / angle_increment)))
        dist_front = np.min(scan_ranges[front_indices])
        if self.debug:
            print(f"Front distance: {dist_front}")
    
        right_index = 0
        right_indices = np.arange(int(right_index - (0.7 / angle_increment)),
                            int(right_index + (0.7 / angle_increment)))
        dist_right = np.min(scan_ranges[right_indices])
        if self.debug:
            print(f"Right distance: {dist_right}")
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time
        pub_msg = Twist()
        if dist_front < 0.45:
            pub_msg.angular.z = 1.0
            pub_msg.linear.x = 0.0
        else:
            z = self.pid(dist_right, dt)
            pub_msg.angular.z = z
            pub_msg.linear.x = 1.0
        self.cmd_pub.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowPIDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()