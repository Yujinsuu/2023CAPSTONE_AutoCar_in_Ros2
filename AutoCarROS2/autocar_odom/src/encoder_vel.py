#! /usr/bin/env python3

import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int32


#50hz로 callback함수 만들
class Pub_Enc_Vel(Node):
    def __init__(self):
        super().__init__('pub_enc_vel')

        qos_profile = QoSProfile(depth=10)
        self.pub_vel = self.create_publisher(Odometry, '/data/encoder_vel', qos_profile)

        self.enc_tic_sub = self.create_subscription(Int32, '/data/encoder_tic', self.get_tic, 10)
        self.steer_sub = self.create_subscription(Float64, '/autocar/now_steer', self.steer_cb, 10)

        self.encoder = 0
        self.time_old = time.time_ns()
        self.wheel_base = 1.040
        self.wheel_tread = 0.985
        self.wheel_radius = 0.265
        self.wheel_pos = 0.0
        self.last_encoder = 0
        self.delta_encoder = 0
        self.delta_pos = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.odom_yaw = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.old_yaw = 0.0
        self.yaw3 = 0.0
        self.i = 0
        self.steer =0.0

        self.encoder_vel = Odometry()

        self.encoder_vel.header.stamp = self.get_clock().now().to_msg()
        self.encoder_vel.header.frame_id = 'odom_footprint'

        self.timer = self.create_timer(0.1, self.pub_encoder_vel)

    def TICK2RAD(self, tic):
      rad = tic*0.06283185307
      return rad

    def get_tic(self, msg):
        #msg.data -2,147,483,648 ~ 2,147,483,647
        self.encoder = msg.data

    def steer_cb(self, msg):
        self.steer = msg.data

    def pub_encoder_vel(self):

        current_time = time.time()
        delta_time=current_time-self.time_old
        #print(delta_time)
        self.time_old=current_time

        self.delta_encoder = self.encoder - self.last_encoder

        self.last_encoder = self.encoder
        if (self.delta_encoder <= -2000000000):
            self.delta_encoder = self.delta_encoder + 2147483647
        elif (self.delta_encoder >= 2000000000):
            self.delta_encoder = self.delta_encoder - 2147483647

        #print("encoder:", self.encoder)

        #print("del end",self.delta_encoder)

        if(self.delta_encoder < 200):
            self.wheel_pos = self.TICK2RAD(self.delta_encoder)
        else: return

        if(self.steer == 0):
            self.encoder_vel.twist.twist.linear.x = self.TICK2RAD(self.delta_encoder) * 0.265 / delta_time
        else:

            L = self.wheel_base / abs(np.tan(self.steer))
            r_center = np.sqrt(pow(L, 2) + pow(self.wheel_base, 2))
            r_in = np.sqrt(pow((L - self.wheel_tread/2), 2) + pow(self.wheel_base, 2))
            r_out = np.sqrt(pow((L + self.wheel_tread/2), 2) + pow(self.wheel_base, 2))
            theta_center = self.steer
            theta_in = np.arctan(self.wheel_tread / (L - self.wheel_tread/2))
            theta_out = np.arctan(self.wheel_tread / (L + self.wheel_tread/2))

            #속도 8
            # speed_up = 1 - (1 - r_center / r_in)*0
            # speed_down = 1 - (1 - r_center / r_out)*0.2

            #속도 5
            speed_up = 1 - (1 - r_center / r_in)
            speed_down = 1 - (1 - r_center / r_out)*1.5

        if self.steer > 0.4:
            self.delta_pos = self.wheel_radius * self.wheel_pos * speed_up /4
            self.get_logger().info('up vel gain: %f' %speed_up)
        elif self.steer < -0.4:
            self.delta_pos = self.wheel_radius * self.wheel_pos * speed_down /4
            self.get_logger().info('down vel gain: %f' %speed_down)
        else:
            self.delta_pos = self.wheel_radius * self.wheel_pos /4

        self.linear_vel = self.delta_pos / delta_time
        self.angular_vel = np.tan(self.steer) * self.linear_vel / self.wheel_base

        # self.get_logger().info('up vel gain: %f' % r_center / r_in)
        # self.get_logger().info('down vel gain: %f' % r_center / r_out)
        #self.vel_now = self.linear_vel

        if self.i == 0:
            self.old_yaw = 0
            self.before_yaw = self.yaw3

        self.odom_yaw = self.old_yaw + self.yaw3
        self.odom_x -= self.delta_pos * np.cos(self.odom_yaw)
        self.odom_y -= self.delta_pos * np.sin(self.odom_yaw)

        self.i += 1

        self.encoder_vel.twist.twist.linear.x = self.linear_vel
        if(self.linear_vel < 10 and self.linear_vel >-10 ):

            self.pub_vel.publish(self.encoder_vel)


def main(args=None):
  rclpy.init(args=args)
  node = Pub_Enc_Vel()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt')
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
	main()
