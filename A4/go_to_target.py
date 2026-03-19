from math import dist

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from enum import Enum
import time
import numpy as np

class State(Enum):
    IDLE = 0
    MOVING_TO_WAYPOINT = 1
    AVOIDING_OBSTACLE = 2
    REACHED_WAYPOINT = 3

class GoToTargetNode(Node):
    def __init__(self):
        super().__init__('go_to_target')
        
        self.waypoints = [[1.5, 0], [1.5, 1.4], [0,1.4]]
        self.current_waypoint_idx = 0
        self.current_pose = None
        self.state = State.IDLE
        self.obstacle_detected = False
        self.latest_scan = None
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        self.max_speed = 0.2
        
        if self.waypoints:
            self.state = State.MOVING_TO_WAYPOINT

        self.tolerance = 0.1

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/fixed_odom', self.odom_callback, 10)
        self.laser_subscription = self.create_subscription(Point, '/object_location', self.laser_callback, 10)
        self.rate = 0.1
        self.loop = self.create_timer(self.rate, self.move_to_waypoint)

    
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
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # print(f"Current position: x={self.current_x:.2f}, y={self.current_y:.2f}")
        self.globalAng = msg.pose.pose.orientation.z
        
    
    def laser_callback(self, msg):
        # self.min_distance = min(msg.ranges) if msg.ranges else float('inf')
        if msg.x == 0 and msg.y == 0:
            self.obstacle_detected = False
        else:
            self.obstacle_detected = True        

        self.obstacle = np.array([msg.x, msg.y])
    
    def move_to_waypoint(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            if self.state != State.REACHED_WAYPOINT:
                self.get_logger().info('All waypoints reached!')
                self.state = State.REACHED_WAYPOINT
            self.stop()
            return

        target = self.waypoints[self.current_waypoint_idx]
        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        dir = np.array([dx, dy])

        # print(f"Moving to waypoint {self.current_waypoint_idx}: target=({target[0]:.2f}, {target[1]:.2f}), dx={dx:.2f}, dy={dy:.2f})")
        distance = np.linalg.norm(dir)

        if self.state == State.MOVING_TO_WAYPOINT:
            if distance < self.tolerance:
                self.get_logger().info(f'Waypoint {self.current_waypoint_idx} reached!')
                self.current_waypoint_idx += 1
                self.state = State.REACHED_WAYPOINT
                self.stop()
                time.sleep(5)  # Wait before moving to next waypoint
                return
            
            if distance > 1e-6:
                dir = dir / distance
            else:
                dir = np.array([1., 0.])

            repulsion_vec = np.array([0., 0.])
            k = 1.0 #TODO tune this gain
            k_rep = 1.5 #TODO tune this gain


            obs_dist = float('inf')
            if self.obstacle_detected:
                obs_dist = np.linalg.norm(self.obstacle)
                if obs_dist > 1e-5 and obs_dist < 0.5:
                    self.get_logger().info(f'Obstacle detected at distance {obs_dist:.2f}, avoiding...')
                    rot_mat = np.array([[np.cos(self.globalAng), -np.sin(self.globalAng)],
                                        [np.sin(self.globalAng), np.cos(self.globalAng)]])
                    obs_global = rot_mat @ self.obstacle
                    repulsion = k_rep * (1/obs_dist - 1/0.5) 
                    
                    obstacle_dir = obs_global / np.linalg.norm(obs_global)
                    repulsion_vec = repulsion * obstacle_dir 
            direction = k * dir + repulsion_vec
            direction_mag = np.linalg.norm(direction)

            if direction_mag > 1e-6:
                heading = np.atan2(direction[1], direction[0])
                heading_error = self.normalize_angle(heading - self.globalAng)
                k_ang = 2.0 #TODO tune this gain
                ang_vel = k_ang * heading_error

                if obs_dist > 0.01 and obs_dist < 0.5:
                    ang_vel/=2

                if distance < 0.25:
                    approach = max(distance*2, 0.25)
                    lin_vel = approach*self.max_speed
                else:
                    lin_vel = self.max_speed

                alignment = max(0, np.cos(heading_error))
                lin_vel *= alignment
                mag = min(1, direction_mag)
                lin_vel *= mag

                if abs(heading_error) > np.pi/4:
                    lin_vel *= 0.5

                if obs_dist> 0.01 and obs_dist < 0.3 and abs(lin_vel) < 0.05 :
                    lin_vel  = 0.05
            else:
                ang_vel = 0.0
                lin_vel = 0.0

            cmd = Twist()
            cmd.linear.x = lin_vel
            cmd.angular.z = ang_vel
            self.publisher_.publish(cmd)
            

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def turn90_degrees(self, clockwise=False):
          # Implement turning logic here in a bit
        cmd = Twist()
        cmd.angular.z = -1.0 if clockwise else 1.0
        self.publisher_.publish(cmd)
        time.sleep(1.9)  # Adjust sleep time based on actual turning speed
        self.stop()  # Stop after turning


    def avoid_obstacle(self, laser_msg):
        self.turn90_degrees(clockwise=True)
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
