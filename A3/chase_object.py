import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')
        
        # Subscribe to object range topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/object_range',
            self.object_callback,
            10
        )
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Control parameters
        self.target_distance = 1.0  # meters
        self.kp_linear = 0.5  # proportional gain for linear velocity
        self.kp_angular = 0.8  # proportional gain for angular velocity
        
        self.get_logger().info('Chase object node started')
    
    def object_callback(self, msg):
        if len(msg.data) < 2:
            return
        
        distance = msg.data[0]  # distance to object in meters
        angle = msg.data[1]     # angle to object in radians
        
        # Create twist message
        twist = Twist()
        
        # Linear velocity controller (maintain 1 meter distance)
        distance_error = distance - self.target_distance
        twist.linear.x = self.kp_linear * distance_error
        
        # Angular velocity controller (keep object centered)
        twist.angular.z = self.kp_angular * angle
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)
        self.get_logger().debug(f'Distance: {distance:.2f}m, Angle: {angle:.2f}rad')


def main(args=None):
    rclpy.init(args=args)
    node = ChaseObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()