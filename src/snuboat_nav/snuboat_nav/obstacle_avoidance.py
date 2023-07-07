# KABOAT
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Int8, Float64, Float64MultiArray
from sensor_msgs.msg import LaserScan
from snuboat_msgs.msg import Obstacles
import numpy as np

class Obstacle_Avoidance(Node):
    def __init__(self): 
        super().__init__('obstacle_avoidance')
        
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
        self.Left_Top = self.declare_parameter("Right_Top", default_params['Right_Top']).value

        self.origin = self.declare_parameter("origin", default_params['origin']).value
        self.dt = 0.1
        self.waypoint = []
        self.cur_wp_idx = 1
        
        self.enu_pos_sub = self.create_subscription(Float64, '/enu_pos', self.enu_pos_callback, 1)
        self.heading_sub = self.create_subscription(Float64, '/heading', self.heading_callback, 1)
        self.spd_sub = self.create_subscription(Float64, '/spd', self.spd_callback, 1)
        self.obstacles_sub = self.create_subscription(Float64MultiArray, '/obstacles', self.obstacles_callback, 1)
        self.enu_wp_set_sub = self.create_subscription(Float64MultiArray, '/enu_wp_set', self.enu_wp_set_callback, 1)

        self.des_heading_pub = self.create_publisher(Float64, '/des_heading', 1)
        self.des_spd_pub = self.create_publisher(Float64, '/des_spd', 1)
        self.cur_wp_idx_pub = self.create_publisher(Int8, '/wp_idx', 1)
        
        self.des_pub = self.create_timer(self.dt, self.pub_des)
        
        self.enu_pos_received = False
        self.heading_received = False
        self.spd_received = False
        self.obstacles_received = False
        self.enu_wp_received = False
        
        # why 10?
        self.enu_pos = np.zeros((10, 2))
        self.enu_wp_set = np.zeros((10,2),dtype = float)
        self.heading = np.zeros(10)
        self.spd = np.zeros(10)
        self.obstacles = []
        self.obstacles_points = []

        self.wp_reach_check = False
        self.wp_time_cnt = 0
        self.goal_tol = 1.5
        self.wp_state = False
        self.wp_stay_time = 30
        
        #
        self.ref_heading = 0
        self.ref_spd = 1
        self.safe_radius = 1.5
        self.safe_heading = []
        self.des_heading = np.zeros(10) # why 10?
        self.des_spd = np.zeros(10)
                
    def wait_for_topics(self):
        self.check_topic_status
        
    def check_topic_status(self):
        if not self.enu_pos_received:
            self.get_logger().info('No topic enu_pos_received')
        if not self.heading_received:
            self.get_logger().info('No topic heading_received')
        if not self.spd_received:
            self.get_logger().info('No topic spd_received')
        if not self.obstacles_received:
            self.get_logger().info('No topic obstacles_received')
        if self.enu_pos_received and self.heading_received and self.spd_received and self.obstacles_received:
            self.get_logger().info('All topics received')
        else: 
            self.get_logger().info('Waiting for topics to be published')
            
    def enu_pos_callback(self, msg):
        self.enu_pos_received = True
        self.enu_pos = np.append(self.enu_pos, [[msg.x, msg.y]], axis=0)
        self.enu_pos = self.enu_pos[1:]

    def enu_wp_set_callback(self,msg):
        self.enu_wp_received = True
        temp_set = np.array(msg.data)
        self.enu_wp_set = np.reshape(temp_set,(int(len(temp_set)/2),2))
        print(self.enu_pos[-1,:])
        print(self.enu_wp_set)
        self.wp_reach_check = np.linalg.norm(self.enu_pos[-1, :]-self.enu_wp_set[self.cur_wp_idx, :]) < self.goal_tol
        if self.wp_reach_check == True:
            if self.wp_state == False:
                self.get_logger().info("Changing waypoint ...")
                self.cur_wp_idx += 1
                self.wp_state = True
            else: # self.wp_state = True
                if self.wp_time_cnt < self.wp_stay_time:
                    self.wp_time_cnt += 1
                    self.wp_state = True
                else: # self.wp_time_cnt > self.wp_stay_time 
                    self.wp_state = False
                    self.wp_time_cnt = 0
        else: # wp_reach_check == False:
            self.wp_state = False            
        

        # Waypoint mission clear check
        if self.cur_wp_idx > len(self.enu_wp_set[:, 0]):
            self.get_logger().info("Waypoint Mission Clear")
            return  

    def heading_callback(self, msg):
        self.heading_received = True
        self.heading = np.append(self.heading, msg.data)
        self.heading = self.heading[1:]
    
    def spd_callback(self, msg):
        self.spd_received = True
        self.spd = np.append(self.spd, msg.data)
        self.spd = self.spd[1:]
        
    def obstacles_callback(self, msg):
        self.obstacles_received = True
        self.obstacles = msg.data
        
    def pub_des(self):        
        des_heading = Float64()
        des_heading.data = self.des_heading[-1]
        self.des_heading_pub.publish(des_heading)
        
        des_spd = Float64()
        des_spd.data = self.des_spd[-1]
        self.des_spd_pub.publish(des_spd) 

        cur_wp_idx_ = Int8()
        cur_wp_idx_.data = self.cur_wp_idx
        self.cur_wp_idx_pub.publish(cur_wp_idx_)
        
    def cal_des(self):
        if self.wp_reach_check == True and self.wp_time_cnt < self.wp_stay_time:
            self.des_spd = np.append(self.des_spd, 0)
            self.des_spd = self.des_spd[1:]
            self.des_heading = np.append(self.des_heading, 0)
            self.des_heading = self.des_heading[1:]
        else: # self.wp_state = False:
            cur_pos = self.enu_pos[-1, :]
            # wp ref heading
            self.err_heading =np.rad2deg(np.arctan2(self.enu_wp_pos[1] - cur_pos[1], self.enu_wp_pos[0] - cur_pos[0]))
            # body fixed {b}
            err_heading = self.err_heading-self.heading
            ##
            if len(self.obstacles) == 0:
                obstacle_num = 0
            else:
                obstacle_num = len(self.obstacles[:, 0])
                
            #### calculate des_heading and des_spd
            if obstacle_num != 0:
                # self.obstacles => [[label r phi x y]]
                for i in range(obstacle_num):
                    # r
                    distance = self.obstacles[i,2]
                    if distance<self.safe_radius:
                        # danger obs
                        continue
                    else:
                        # safe obs => save [phi-ref_heading] radian
                        obs_heading_b = self.obstacles[i,3]-math.pi/2
                        diff_heading = abs(obs_heading_b-err_heading)
                        self.safe_heading = np.append(self.safe_heading,diff_heading,axis=0)
    
                self.des_spd = np.append(self.des_spd, self.ref_spd)
                self.des_spd = self.des_spd[1:]
                self.des_heading = np.append(self.des_heading, np.min(self.safe_heading))
                self.des_heading = self.des_heading[1:]
            else:
                self.des_spd = np.append(self.des_spd, self.des_spd[-1])
                self.des_spd = self.des_spd[1:]
                self.des_heading = np.append(self.des_heading, self.des_heading[-1])
                self.des_heading = self.des_heading[1:]
        
def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = Obstacle_Avoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()        
    
