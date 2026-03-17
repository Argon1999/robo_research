import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from enum import Enum
import time
import math

class State(Enum):
    IDLE = 0
    MOVING_TO_WAYPOINT = 1
    AVOIDING_OBSTACLE = 2
    REACHED_WAYPOINT = 3

class GoToTargetNode(Node):
    def __init__(self):
        super().__init__('go_to_target')
        
        self.waypoints = self.load_waypoints()
        self.current_waypoint_idx = 0
        self.current_pose = None
        self.state = State.IDLE
        self.obstacle_detected = False
        self.min_distance = float('inf')
        self.latest_scan = None
        
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        
        if self.waypoints:
            self.state = State.MOVING_TO_WAYPOINT

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_subscription = self.create_subscription(Point, '/object_location', self.laser_callback, 10)
        
    
    def load_waypoints(self):
        waypoints = []
        try:
            with open('wayPoints.txt', 'r') as f:
                for line in f:
                    coords = line.strip().split()
                    if len(coords) == 2:
                        waypoints.append((float(coords[0]), float(coords[1])))
        except FileNotFoundError:
            self.get_logger().error('wayPoints.txt not found')
        return waypoints
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        
        if self.state == State.MOVING_TO_WAYPOINT:
            self.move_to_waypoint()
        elif self.state == State.AVOIDING_OBSTACLE:
            if not self.obstacle_detected:
                self.state = State.MOVING_TO_WAYPOINT
    
    def laser_callback(self, msg):
        # self.min_distance = min(msg.ranges) if msg.ranges else float('inf')
        self.latest_scan = msg
        self.front_distance = msg.data[0] if msg.data else float('inf')

        if State.AVOIDING_OBSTACLE:
            return
        
        # if self.min_distance < 0.3:
        #     if self.state == State.MOVING_TO_WAYPOINT:
        #         self.state = State.AVOIDING_OBSTACLE
        #         self.obstacle_detected = True
        #     self.avoid_obstacle(msg)
        # else:
        #     self.obstacle_detected = False
    
    def move_to_waypoint(self):
        if self.state == State.AVOIDING_OBSTACLE:
            return
        
        if self.current_waypoint_idx >= len(self.waypoints):
            self.stop()
            return

        target = self.waypoints[self.current_waypoint_idx]
        dx = target[0] - self.current_pose.x
        dy = target[1] - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:
            self.current_waypoint_idx += 1
            self.state = State.REACHED_WAYPOINT
            self.turn90_degrees()
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx}')
            time.sleep(5)  # Wait before moving to next waypoint
            return
        
        if self.front_distance < 0.3:
            self.state = State.AVOIDING_OBSTACLE
            self.avoid_obstacle()


        else:
            cmd = Twist()
            cmd.linear.x = min(0.5, distance * 0.3)
            cmd.angular.z = math.atan2(dy, dx) * 0.5
            self.publisher_.publish(cmd)
    
    def turn90_degrees(self, clockwise=False):
          # Implement turning logic here in a bit
        cmd = Twist()
        cmd.angular.z = -1 if clockwise else 1
        self.publisher_.publish(cmd)
        time.sleep(2)  # Adjust sleep time based on actual turning speed


    def avoid_obstacle(self, laser_msg):
        self.turn90_degrees(clockwise=True)
        time.sleep(2)  # Adjust sleep time based on actual turning speed
        cmd = Twist()
        cmd.linear.x = 0.5
        self.publisher_.publish(cmd)
        time.sleep(3)  # Move forward for a short time to bypass obstacle
        self.obstacle_detected = False
        self.state = State.MOVING_TO_WAYPOINT

    
    def stop(self):
        cmd = Twist()
        self.publisher_.publish(cmd)
        self.state = State.IDLE

def main(args=None):
    rclpy.init(args=args)
    node = GoToTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()