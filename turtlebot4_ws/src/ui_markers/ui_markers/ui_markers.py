import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray


class UIMarkers(Node):
    def __init__(self):
        super().__init__('ui_markers')
        
        # Subscribe to PoseArray from map_annotator
        self.pose_array_sub = self.create_subscription(
            PoseArray,
            '/saved_poses',  # Adjust topic name if needed
            self.pose_array_callback,
            10
        )
        
        # Publisher for markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            10
        )
        
        self.get_logger().info('UI Markers node started')
        
    def pose_array_callback(self, msg):
        """Process incoming PoseArray and create markers"""
        marker_array = MarkerArray()
        
        # Clear previous markers first
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Create markers for each pose
        for i, pose in enumerate(msg.poses):
            # Create arrow marker for pose direction
            arrow_marker = self.create_arrow_marker(i, pose, msg.header)
            marker_array.markers.append(arrow_marker)
        
        # Publish all markers
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(msg.poses)} waypoint markers')
    
    def create_arrow_marker(self, idx, pose, header):
        """Create an arrow marker showing pose orientation"""
        marker = Marker()
        marker.header = header
        marker.ns = "arrows"
        marker.id = idx
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Set pose
        marker.pose = pose
        
        # Set scale (x=length, y=width, z=height)
        marker.scale.x = 0.5  # Arrow length
        marker.scale.y = 0.1  # Arrow width
        marker.scale.z = 0.1  # Arrow height

        marker.pose.orientation.w = 1.0  # Ensure marker is oriented correctly
        
        # Set color (blue arrows)
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        marker.lifetime.sec = 0  # Persist until deleted
        
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = UIMarkers()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()