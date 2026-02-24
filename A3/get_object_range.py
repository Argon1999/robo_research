import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from geometry_msgs.msg import Float32MultiArray

import math

class ObjectRangeNode(Node):
    def __init__(self):
        super().__init__('object_range_node')
        
        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.object_subscriber = self.create_subscription(
            Point,
            '/object_location',
            self.object_callback,
            10
        )
        
        # Publisher
        self.range_publisher = self.create_publisher(
            Float32MultiArray,
            '/object_range',
            10
        )
        
        self.latest_scan = None
        self.object_info = None
    
    def scan_callback(self, msg: LaserScan):
        """Store latest lidar scan data"""
        self.latest_scan = msg
    
    def object_callback(self, msg: Point):
        """Process detected object and calculate range/angle"""
        self.x = msg.x
        self.y = msg.y
        self.frame_width = msg.z
        
        if self.latest_scan is None:
            return
        
        # Assuming object_info contains [x, y] or angle index
        # Extract angle from object detection (adjust based on your detect_object output)
        angle_idx = int(self.object_info[0])
        
        # Get range from lidar scan at detected angle
        if angle_idx < len(self.latest_scan.ranges):
            distance = self.latest_scan.ranges[angle_idx]
            angle = self.latest_scan.angle_min + (angle_idx * self.latest_scan.angle_increment)
            
            # Publish result: [distance, angle]
            result = Float32MultiArray(data=[float(distance), float(angle)])
            self.range_publisher.publish(result)
            
            self.get_logger().info(f'Distance: {distance:.2f}m, Angle: {math.degrees(angle):.2f}°')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectRangeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()