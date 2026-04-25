import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import pickle
import os
import glob
import math
import time
import random
import threading # Added for sound thread
from collections import Counter
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class MazeSolve(Node):
    def __init__(self):
        super().__init__('maze_solve_node')
        
        # Load the trained model
        model_path = os.path.join(get_package_share_directory('maze_solver'), 'model.pkl')
        if not os.path.isfile(model_path):
            self.get_logger().error(f"Model file not found: {model_path}")
            raise FileNotFoundError(f"Model file not found: {model_path}")
        
        with open(model_path, 'rb') as f:
            self.model = pickle.load(f)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Create subscribers and publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.image_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, qos_profile)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.current_image = None
        self.current_scan = None
        self.current_odom = None
        self.current_yaw = None
        self.last_decision_time = time.time()
        self.decision_interval = 0.5  # seconds

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz control loop
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img = cv2.resize(cv_image, (320, 240))  # Resize to match training data
            if cv_image is not None:
                self.current_image = cv_image
            else:
                self.get_logger().warning("Failed to decode image")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def scan_callback(self, msg):
        self.current_scan = msg
    
    def odom_callback(self, msg):
        self.current_odom = msg
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        ranges = self.current_scan.ranges
        n = len(ranges)
        if n < 100:
            return  # Not enough scan data

        
        

    def normalize_angle(self, angle):
        if angle > math.pi:
            angle -= 2*math.pi
        elif angle < -math.pi:
            angle += 2*math.pi
        return angle
    
