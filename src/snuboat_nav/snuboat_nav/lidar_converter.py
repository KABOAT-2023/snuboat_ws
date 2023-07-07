# KABOAT
import rclpy
import os
import yaml
from math import pi, cos, sin
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float64, Float64MultiArray
from sensor_msgs.msg import LaserScan
from snuboat_msgs.msg import Obstacles
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

class Lidar_Converter(Node):
    def __init__(self):
        super().__init__('lidar_converter')
        
        default_params = {
            
        }
        self.dt = 0.1
        self.obstacle = [] 
        self.obstacles = []
        self.cartesian_scan = np.empty((0,2),float) # origin: boat
        
        self.lidar_scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_scan_callback, qos_profile_sensor_data)
        
        self.obstacles_pub = self.create_publisher(Float64MultiArray, '/obstacles', 1)
        
        self.obstacles_timer = self.create_timer(self.dt, self.pub_obstacles)
        
        #publish => (labels, r, phi)
        # [[labels, r1,phi1]
        #  [label2, r2,phi2]]
        self.polar_pub =np.empty((0,3),int)

        self.lidar_scan_received = False
        
    def wait_for_topics(self):
        self.check_topic_status
        
    def check_topic_status(self):
        if not self.lidar_scan_received:
            self.get_logger().info('Topic lidar_scan not received')
        if not self.lidar_scan_received:
            self.get_logger().info('Waiting for topics to be published')
        else:
            self.get_logger().info('All topics received')
    
    # scan msg => save in polar_pub
    def lidar_scan_callback(self, msg):
        self.lidar_scan_received = True
        #temporary polar coord []
        temp_polar = np.empty((0,2),float)
        phi = msg.angle_min # radians
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                temp_polar = np.append(temp_polar,[[r,phi]],axis=0)
                #Point => polar_to_cartesian not exist 
                p = [r*cos(phi),r*sin(phi)]
                self.cartesian_scan = np.append(self.cartesian_scan, [p], axis = 0)
            phi += msg.angle_increment
        print(self.cartesian_scan)
        print(len(self.cartesian_scan))
        print(msg.angle_increment)
        points = self.cartesian_scan
        scaler = StandardScaler()
        points_scaled = scaler.fit_transform(points)
        dbscan = DBSCAN(eps=0.1, min_samples=5)  # Adjust the parameters as per your data
        dbscan.fit(points_scaled)
        self.scan_labels = dbscan.labels_
        print(self.scan_labels)
        print(temp_polar)
        # append label to polar coord
        for i in range(len(temp_polar)):
            coord = temp_polar[i]
            coord = np.insert(coord,0,self.scan_labels[i])
            print("label added")
            print(coord)
            self.polar_pub = np.append(self.polar_pub,[coord],axis=0)
        
    # publish obstacle sinfo
    def pub_obstacles(self):
        self.obstacles = np.zeros((5,max(self.scan_labels)))
        if max(self.scan_labels) != -1:
            for i in range(max(self.scan_labels)):
                idxs = np.where(self.scan_labels == i)[0]
                self.obstacle = self.cartesian_scan[idxs, :]
                self.obstacle = np.reshape(self.obstacle, (1, -1))
                # [[label r phi x y]]
                self.obstacle = np.insert(self.obstacle,0,self.polar_pub[i])
                self.obstacles[i] = self.obstacle # => maybe error? 
        
        # else: self.obstacles = []
            
        obs = Float64MultiArray()
        obs.data = self.obstacles
        #polar_pub 추가

        self.obstacles_pub.publish(obs)
        
def main(args=None):
    rclpy.init(args=args)
    lidar_converter = Lidar_Converter()
    rclpy.spin(lidar_converter)
    lidar_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
