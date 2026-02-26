import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import time

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

        self.timer = self.create_timer(1, self.timer_callback)  # 10 Hz timer for timeout handling
        
        # Control parameters
        self.target_distance = 0.3  # meters
        self.kp_angular = 0.8  # proportional gain for angular velocity
        
        self.get_logger().info('Chase object node started')
        self.time = time.time()

    def timer_callback(self):
        # If no message received within timeout, stop the robot
        if time.time() - self.time > 1.0:  # 1 second timeout
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
        return
    
    def object_callback(self, msg):
        if len(msg.data) < 2:
            return
        
        distance = msg.data[0]  # distance to object in meters
        angle = msg.data[1]     # angle to object in radians
        
        # Create twist message
        twist = Twist()
        
        # Linear velocity controller (maintain 1 meter distance)
        # PID controller for linear velocity
        if not hasattr(self, 'distance_error_prev'):
            self.distance_error_prev = 0
            self.distance_error_integral = 0
        
        distance_error = distance - self.target_distance
        self.distance_error_integral += distance_error
        distance_error_derivative = distance_error - self.distance_error_prev
        self.distance_error_prev = distance_error
        
        kp, ki, kd = 0.5, 0.0, 0.0
        twist.linear.x = kp * distance_error + ki * self.distance_error_integral + kd * distance_error_derivative
        
        # Angular velocity controller (keep object centered)
        # Angular velocity controller (keep object centered with PID)
        if not hasattr(self, 'angle_error_prev'):
            self.angle_error_prev = 0
            self.angle_error_integral = 0
        
        angle_error = angle 
        self.angle_error_integral += angle_error
        angle_error_derivative = angle_error - self.angle_error_prev
        self.angle_error_prev = angle_error
        
        kp_a, ki_a, kd_a = 0.8, 0.0, 0.0
        twist.angular.z = -(kp_a * angle_error + ki_a * self.angle_error_integral + kd_a * angle_error_derivative)
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)
        self.time = time.time()
        print(f'{twist.linear.x=}m/s, {twist.angular.z=}rad/s')


def main(args=None):
    rclpy.init(args=args)
    node = ChaseObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()