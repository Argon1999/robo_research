import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import math

class ObjectRange(Node):
    def __init__(self):
        super().__init__('object_range')
        
        qos = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1)
        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos
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

        # Calculate angle to object based on frame width and lidar scan parameters
        angle_increment = self.latest_scan.angle_increment
        angle_min = self.latest_scan.angle_min
        angle_max = self.latest_scan.angle_max
        num_ranges = len(self.latest_scan.ranges)
        # print(f"{angle_min=}, {angle_max=}, {angle_increment=}")
        # Calculate the angle corresponding to the detected object
        # print(f"{self.x=}, {self.frame_width=}")
        object_angle = (self.x / self.frame_width) * 1.0855 - 0.52
        # Find the closest range measurement at the calculated angle
        index = -int((object_angle) / angle_increment)
        # print(f"{len(self.latest_scan.ranges)=}, {angle_min=}, {angle_max=}, {angle_increment=}")
        if index < 0:
            index = len(self.latest_scan.ranges) + index

        # print(f"{index=}")
        if 0 <= index < num_ranges:
            object_range = self.latest_scan.ranges[index]
            # Publish the range and angle as a Float32MultiArray
            if object_range == float('inf') or math.isnan(object_range):
                return
            range_msg = Float32MultiArray()
            range_msg.data = [object_range, object_angle]
            self.range_publisher.publish(range_msg)
            print(f"{object_range=}, {object_angle=}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()