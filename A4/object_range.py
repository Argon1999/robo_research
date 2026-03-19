import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Vector3
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np

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
          
        # Publisher
        self.range_publisher = self.create_publisher(
            Vector3,
            '/object_location',
            10
        )
        
        self.latest_scan = None
        self.object_info = None
    
    def scan_callback(self, msg: LaserScan):
        """Store latest lidar scan data"""
        self.latest_scan = msg

        if len(msg.ranges) == 0:
            self.get_logger().warn("Received empty scan data")
            self.pubish_zero_vector()
            return
        # Calculate angle to object based on frame width and lidar scan parameters
        angle_increment = self.latest_scan.angle_increment
        angle_min = self.latest_scan.angle_min
        angle_max = self.latest_scan.angle_max
        num_ranges = len(self.latest_scan.ranges)
        # print(f"{angle_min=}, {angle_max=}, {angle_increment=}")
        # Calculate the angle corresponding to the detected object
        # print(f"{self.x=}, {self.frame_width=}")
        front_60 = np.radians(60)
        # Calculate the index range for the front 60 degrees
        index_60 = int((front_60 - angle_min) / angle_increment)
        pts = self.get_valid_points(index_60, angle_min, angle_increment, num_ranges)
        print(f"Valid points: {len(pts)}")

        clusters = self.cluster_points(pts)
        if not clusters:
            self.get_logger().info("No valid clusters found")
            self.pubish_zero_vector()
            return
        
        filtered_clusters = [cluster for cluster in clusters if len(cluster) >= 3 and self.is_not_wall(cluster)]
        if not filtered_clusters:
            # print(clusters)
            self.get_logger().info("No clusters with enough points found")
            self.pubish_zero_vector()
            return
        
        # Find the closest cluster
        closest_cluster = min(filtered_clusters, key=lambda c: self.get_cluster_distance(c))
        x, y = self.get_cluster_center(closest_cluster)
        print(f"Closest object at x={x:.2f}, y={y:.2f}")

        # Publish the object location
        object_location = Vector3()
        object_location.x = x
        object_location.y = y
        object_location.z = 0.0
        self.range_publisher.publish(object_location)
        
    def is_not_wall(self, cluster):
        # Check if the cluster is likely a wall by analyzing its shape and size
        if len(cluster) < 2:
            return False
        
        x_coords = [point[0] for point in cluster]
        y_coords = [point[1] for point in cluster]
        width = max(x_coords) - min(x_coords)
        height = max(y_coords) - min(y_coords)
        
         # Likely a wall
        return np.sqrt(width**2 + height**2) < 1.0  # Not a wall

    def get_cluster_center(self, cluster):
        x_coords = sum([point[0] for point in cluster])/ len(cluster)
        y_coords = sum([point[1] for point in cluster])/ len(cluster)
        return x_coords, y_coords
    
    def get_cluster_distance(self, cluster):
        x,y = self.get_cluster_center(cluster)
        return np.sqrt(x**2 + y**2)

    def get_valid_points(self, index_60, angle_min, angle_increment, num_ranges):
        pts = []

        distance = self.latest_scan.ranges[0]
        if not np.isinf(distance) and not np.isnan(distance) and distance > 0.05 and distance < 3:
            pts.append((distance, 0.0, 0, distance))

        #left 60 deg of the lidar scan
        for i in range(1, index_60):
            distance = self.latest_scan.ranges[i]
            if np.isinf(distance) or np.isnan(distance):
                continue                
            
            if distance < 0.05 or distance > 3:
                continue

            angle = angle_min + i * angle_increment
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            pts.append((x, y, i, distance))

        for i in range(num_ranges - index_60, num_ranges):
            distance = self.latest_scan.ranges[i]
            if np.isinf(distance) or np.isnan(distance):
                continue                
            
            if distance < 0.05 or distance > 3:
                continue

            angle = angle_min + (i-num_ranges) * angle_increment
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            pts.append((x, y, i, distance))
        return pts

    def cluster_points(self, pts):
        clusters = []
        distance_threshold = 0.2 
        pts = sorted(pts, key=lambda p: p[2]) 
        current_cluster = [pts[0]]


        for i in range(1, len(pts)):
            if not current_cluster:
                current_cluster.append(pts[i])
            else:
                prev_point = current_cluster[-1]
                curr_point = pts[i]
                distance = np.sqrt((curr_point[0] - prev_point[0])**2 + (curr_point[1] - prev_point[1])**2)
                
                if distance < distance_threshold:
                    current_cluster.append(curr_point)
                else:
                    clusters.append(current_cluster)
                    current_cluster = [curr_point]

        if current_cluster:
            clusters.append(current_cluster)

        return clusters

    def pubish_zero_vector(self):
        zero_vector = Vector3()
        zero_vector.x = 0.0
        zero_vector.y = 0.0
        zero_vector.z = 0.0
        self.range_publisher.publish(zero_vector)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()