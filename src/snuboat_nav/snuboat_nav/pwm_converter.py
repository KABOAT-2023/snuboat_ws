# KABOAT
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float64, Int32
import pymap3d as pm
import numpy as np

class PWMConverter(Node):
    def __init__(self):
        super().__init__('pwm_converter')

        # config_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'pwm_config.yaml')
        # with open(config_file, 'r') as file:
        #     config = yaml.safe_load(file)
        #     self.thrust_lim = config['thrust_lim']
        #     self.servo_lim = config['servo_lim']
        #     self.pwm_lim = config['pwm_lim']
        #     self.dt = config['dt']
        #     self.Kp_thrust = config['Kp_thrust']
        #     self.Kd_thrust = config['Kd_thrust']
        #     self.Kp_servo = config['Kp_servo']
        #     self.Kd_servo = config['Kd_servo']
        default_params = {
            'rudder_lim' : [-50, 50],
            # rudder pwm 500~2500 270 deg
            'rudder_pwm_lim' : [500, 2500], # 1000: left, 2000: right
            'thrust_pwm_lim' : [1500, 1650], # 1500: stop
            'dt' : 0.1, 
            'Kp_thrust' : 150.0,
            'Kd_thrust' : 50.0,
            'Kp_rudder' : 1.0,
            'Kd_rudder' : 0.2,
        }
        self.rudder_lim = self.declare_parameter("rudder_lim", default_params['rudder_lim']).value
        self.rudder_pwm_lim = self.declare_parameter("rudder_pwm_lim", default_params['rudder_pwm_lim']).value
        self.thrust_pwm_lim = self.declare_parameter("thrust_pwm_lim", default_params['thrust_pwm_lim']).value
        self.dt = self.declare_parameter("dt", default_params['dt']).value
        self.Kp_thrust = self.declare_parameter("Kp_thrust", default_params['Kp_thrust']).value
        self.Kd_thrust = self.declare_parameter("Kd_thrust", default_params['Kd_thrust']).value
        self.Kp_rudder = self.declare_parameter("Kp_rudder", default_params['Kp_rudder']).value
        self.Kd_rudder = self.declare_parameter("Kd_rudder", default_params['Kd_rudder']).value
            
        #namespace_OS = '/OS'
        namespace_OS = ''
        
        self.OS_des_heading = np.zeros(10)
        self.OS_des_spd = np.zeros(10)
        self.OS_heading = np.zeros(10)
        self.OS_spd = np.zeros(10)
        
        self.err_heading = np.zeros(10)
        self.err_spd = np.zeros(10)
        
        self.thrust_pwm = np.zeros(10)
        self.rudder_pwm = np.zeros(10)
        
        self.OS_des_heading_sub = self.create_subscription(Float64, namespace_OS + '/des_heading', self.OS_des_heading_callback, 1)
        self.OS_des_spd_sub =self.create_subscription(Float64, namespace_OS + '/des_spd', self.OS_des_spd_callback, 1)
        self.OS_heading = self.create_subscription(Float64, namespace_OS + '/heading', self.OS_heading_callback, 1)
        self.OS_spd = self.create_subscription(Float64, namespace_OS + '/spd', self.OS_spd_callback, 1)
        
        self.pwm_pub =self.create_publisher(Int32, namespace_OS + '/pwm', 1)
        
        self.pwm_timer = self.create_timer(self.dt, self.pub_pwm)

        self.OS_des_heading_received = False
        self.OS_des_spd_received = False
        self.OS_heading_received = False
        self.OS_spd_received = False

    def wait_for_topics(self):
        self.timer = self.create_timer(1.0, self.check_topics_status)

    def check_topics_status(self):
        if not self.OS_des_heading_received:
            self.get_logger().info('No topic OS_des_heading_received')
        if not self.OS_des_spd_received:
            self.get_logger().info('No topic OS_des_spd_received')
        if not self.OS_heading_received:
            self.get_logger().info('No topic OS_heading_received')
        if not self.OS_spd_received:
            self.get_logger().info('No topic OS_spd_received')
        if self.OS_des_heading_received and self.OS_des_spd_received and self.OS_heading_received and self.OS_spd_received:
            self.get_logger().info('All topics received')
        else:
            self.get_logger().info('Waiting for topics to be published...')

    def OS_des_heading_callback(self, msg):
        self.OS_des_heading_received = True
        self.OS_des_heading = np.append(self.OS_des_heading, msg.data)
        self.OS_des_heading = self.OS_des_heading[1:]
        
    def OS_des_spd_callback(self, msg):
        self.OS_des_spd_received = True
        self.OS_des_spd = np.append(self.OS_des_spd, msg.data)
        self.OS_des_spd = self.OS_des_spd[1:]
        print(self.OS_des_spd[-1])
        
    def OS_heading_callback(self, msg):
        self.OS_heading_received = True
        self.OS_heading = np.append(self.OS_heading, msg.data)
        self.OS_heading = self.OS_heading[1:]
        
        err_hdg = self.OS_des_heading[-1] - self.OS_heading[-1]
        if err_hdg > 180:
            err_hdg -= 360
        elif err_hdg < -180:
            err_hdg += 360
            
        self.err_heading = np.append(self.err_heading, err_hdg)
        self.err_heading = self.err_heading[1:]
        
    def OS_spd_callback(self, msg):
        self.OS_spd_received = True
        self.OS_spd = np.append(self.OS_spd, msg.data)
        self.OS_spd = self.OS_spd[1:]
        
        err_spd = self.OS_des_spd[-1] - self.OS_spd[-1]
        # print(err_spd)
        self.err_spd = np.append(self.err_spd, err_spd)
        self.err_spd = self.err_spd[1:]
        
    def pub_pwm(self):
        self.cal_thrust_pwm()
        self.cal_rudder_pwm()

        thrust_pwm_str = str(int(self.thrust_pwm[-1])).zfill(4)
        servo_pwm_str = str(int(self.rudder_pwm[-1])).zfill(4)
        pwm_value = int(thrust_pwm_str + servo_pwm_str)

        pwm = Int32()
        pwm.data = pwm_value
        self.pwm_pub.publish(pwm)
        
    # functions that return something
    def cal_thrust_pwm(self):
        thrust_pwm = self.Kp_thrust * self.err_spd[-1] - self.Kd_thrust * self.err_spd[-2] + \
            (self.thrust_pwm_lim[0] + self.thrust_pwm_lim[1]) / 2
        if thrust_pwm > self.thrust_pwm_lim[1]:
            thrust_pwm = self.thrust_pwm_lim[1]
        elif thrust_pwm < self.thrust_pwm_lim[0]:
            thrust_pwm = self.thrust_pwm_lim[0]

        # print(self.err_spd[-1], ":", self.err_spd[-2])

        thrust_pwm = int(thrust_pwm)
        
        self.thrust_pwm = np.append(self.thrust_pwm, thrust_pwm)
        self.thrust_pwm = self.thrust_pwm[1:]
    
    def cal_rudder_pwm(self):
        rudder = -(self.Kp_rudder * self.err_heading[-1] - self.Kd_rudder * self.err_heading[-2])
        if rudder > self.rudder_lim[1]:
            rudder = self.rudder_lim[1]
        elif rudder < self.rudder_lim[0]:
            rudder = self.rudder_lim[0]
        norm_rudder = (rudder - self.rudder_lim[0]) / (self.rudder_lim[1] - self.rudder_lim[0])
        r_pwm = self.rudder_pwm_lim[0] + (self.rudder_pwm_lim[1] - self.rudder_pwm_lim[0]) * norm_rudder
        r_pwm = int(r_pwm)
        
        self.rudder_pwm = np.append(self.rudder_pwm, r_pwm)
        self.rudder_pwm = self.rudder_pwm[1:]
    
def main(args=None):
    rclpy.init(args=args)
    pwm_converter = PWMConverter()
    rclpy.spin(pwm_converter)
    pwm_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
