# KABOAT
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float64MultiArray
import pymap3d as pm
import numpy as np

class GNSSConverter(Node):
    def __init__(self):
        super().__init__('gnss_converter')

        # config_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'gnss_config.yaml')
        # with open(config_file, 'r') as file:
        #     config = yaml.safe_load(file)
        #     self.origin = self.get_parameter('origin').value
        default_params = {
            # to be modified
            'Left_Bottom' : [37.4557583, 126.9517448], # to be modified
            'Right_Bottom' : [37.4558121667, 126.9517401667],
            'Left_Top' : [37.4556311667, 126.9518098333], # to be modified
            'Right_Top' : [37.4556311667, 126.9518098333], # to be modified
            'origin' : [37.4557583, 126.9517448], # to be modified, same as Left_Bottom
        }
        self.Left_Bottom = self.declare_parameter("Left_Bottom", default_params['Left_Bottom']).value
        self.Right_Bottom = self.declare_parameter("Right_Bottom", default_params['Right_Bottom']).value
        self.Left_Top = self.declare_parameter("Left_Top", default_params['Left_Top']).value
        self.Right_Top = self.declare_parameter("Right_Top", default_params['Right_Top']).value

        self.origin = self.declare_parameter("origin", default_params['origin']).value

        self.wp_set = np.vstack((self.Right_Top, self.Left_Top, self.Right_Bottom))

        self.pos = [0, 0, 0]

        self.gps_lon_sub = self.create_subscription(String, '/gps/lon', self.gps_lon_callback, 1)
        self.gps_lat_sub = self.create_subscription(String, '/gps/lat', self.gps_lat_callback, 1)
        
        self.enu_pos_pub = self.create_publisher(Point, '/enu_pos', 1)
        self.enu_wp_set_pub = self.create_publisher(Float64MultiArray, '/enu_wp_set', 1)

        self.OS_timer = self.create_timer(0.1, self.pub_enu_pos)

        self.enu_pos_log_pub = self.create_publisher(String, '/log/enu_pos', 1)
       
        self.gps_lon_received = False
        self.gps_lat_received = False
        self.waypoint_received = False

    def wait_for_topics(self):
        self.timer = self.create_timer(1.0, self.check_topics_status)

    def check_topics_status(self):
        if not self.gps_lat_received:
            self.get_logger().info('No topic gps_lat_received')
        if not self.gps_lon_received:
            self.get_logger().info('No topic gps_lon_received')
        if self.gps_lat_received and self.gps_lon_received:
            self.get_logger().info('All topics received')
        else:
            self.get_logger().info('Waiting for topics to be published...')

    def gps_lon_callback(self, msg):
        self.gps_lon_received = True
        str_gps_lon = msg.data
        self.gps_lon = float(str_gps_lon)

    def gps_lat_callback(self, msg):
        self.gps_lat_received = True
        str_gps_lat = msg.data
        self.gps_lat = float(str_gps_lat)

    def waypoint_callback(self, msg):
        self.waypoint_received = True
        self.wp_lat = msg.x
        self.wp_lon = msg.y

    def enu_convert(self, gnss):
        e, n, u = pm.geodetic2enu(gnss[0], gnss[1], 0, self.origin[0], self.origin[1], 0)
        return e, n, u
    
    def pub_enu_pos(self):
        if self.gps_lat_received and self.gps_lon_received:
            self.pos[0], self.pos[1], self.pos[2] = self.enu_convert([self.gps_lat, self.gps_lon])
            enu_pos = Point()
            enu_pos.x = self.pos[0]
            enu_pos.y = self.pos[1]
            self.enu_pos_pub.publish(enu_pos)
            
            enu_pos_log = String()
            enu_pos_log.data = str(self.pos[0]) + "," + str(self.pos[1])
            self.enu_pos_log_pub.publish(enu_pos_log)

        self.enu_wp_set = np.zeros((len(self.wp_set[:, 0]), 3))
        for i in range(len(self.wp_set[:, 0])):
            self.enu_wp_set[i, 0], self.enu_wp_set[i, 1], self.enu_wp_set[i, 2] = self.enu_convert([self.wp_set[i, 0], self.wp_set[i, 1]])
            
        enu_wp_set = Float64MultiArray()
        temp_wp_set = self.enu_wp_set[:, 0:2].reshape([2*len(self.enu_wp_set[:, 0:2])])
        #numpy float error!!
        enu_wp_set.data = temp_wp_set.tolist()
        self.enu_wp_set_pub.publish(enu_wp_set)
        


def main(args=None):
    rclpy.init(args=args)
    gnss_converter = GNSSConverter()
    gnss_converter.wait_for_topics()
    rclpy.spin(gnss_converter)
    gnss_converter.destroy_node()
    rclpy.shutodown()

if __name__ == '__main__':
    main()
 
