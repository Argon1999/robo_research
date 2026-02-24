import geometry_msgs
import rclpy
from rclpy.node import Node

class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            geometry_msgs.msg.Point,
            '/object_location',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        twist = geometry_msgs.msg.Twist()
        if msg.x < msg.z/2-10:  # Assuming image width is 640
            twist.angular.z = 0.5  # Rotate left
        elif msg.x > msg.z/2 + 10:
            twist.angular.z = -0.5  # Rotate right
        else:
            twist.angular.z = 0.0  # Stop rotation
        self.publisher_.publish(twist)

def main():
    rclpy.init()  # Initialize ROS2
    rotate_robot = RotateRobot()  # Create an instance of RotateRobot

    while rclpy.ok():
        rclpy.spin_once(rotate_robot)  # Process callbacks

    rclpy.logging.get_logger("Rotate Robot Node Info...").info("Shutting Down")
    rotate_robot.destroy_node()  # Clean up and shutdown
    rclpy.shutdown()
