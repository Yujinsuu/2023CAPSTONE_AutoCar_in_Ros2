#! /usr/bin/env python3

import time
import math as m

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped


class LowPassFilter:
    def __init__(self, cutoff_freq, update_rate):
        self.update_rate = update_rate
        self.alpha = cutoff_freq / (cutoff_freq + update_rate)
        self.filtered_angle = 0.0

    def update(self, input_angle):
        self.filtered_angle += self.alpha * (input_angle - self.filtered_angle)

        return self.filtered_angle


class Pub_Two_Enc_Vel2(Node):
    def __init__(self):
        super().__init__('pub_enc_tic')

        qos_profile = QoSProfile(depth=10)
        self.pub_enc_vel_two = self.create_publisher(Odometry, '/data/encoder_vel_two', qos_profile)
        self.pub_enc_vel_one = self.create_publisher(Odometry, '/data/encoder_vel_one', qos_profile)

        self.steer_sub = self.create_subscription(AckermannDriveStamped, '/autocar/autocar_cmd', self.steer_callback, 10)
        self.sub_enc_tic = self.create_subscription(Int32MultiArray, '/data/encoder_tic',self.get_enc_tic, qos_profile)


        self.encoder_tic_left = None
        self.encoder_tic_right = None

        self.encoder_vel_two = Odometry()
        self.encoder_vel_two.header.stamp = self.get_clock().now().to_msg()
        self.encoder_vel_two.header.frame_id = 'odom_footprint'

        self.encoder_vel_one = Odometry()
        self.encoder_vel_one.header.stamp = self.get_clock().now().to_msg()
        self.encoder_vel_one.header.frame_id = 'odom_footprint'


        self.time_old = 0.0
        self.old_enc_left = 0
        self.old_enc_right = 0

        self.wheel_base = 1.040
        self.wheel_tread = 0.985
        self.wheel_radius = 0.26 # 0.265
        self.wheel_pos3 = 0.0
        self.last_encoder3 = 0
        self.delta_encoder3 = 0
        self.delta_pos3 = 0.0
        self.linear_vel3 = 0.0
        self.angular_vel3 = 0.0
        self.odom_yaw3 = 0.0
        self.odom_x3 = 0.0
        self.odom_y3 = 0.0
        self.old_yaw3 = 0.0
        self.yaw3 = 0.0
        self.i3 = 0
        self.time_old3 = 0.0

        self.steer =0.0
        self.filter = LowPassFilter(cutoff_freq=4.3, update_rate=10.0)

        self.j =0

        self.timer = self.create_timer(0.1, self.data_callback)

    def get_enc_tic(self,msg):

        self.encoder_tic_left = msg.data[0]
        self.encoder_tic_right = msg.data[1]
        print(self.encoder_tic_left)

    def pub_encoder_vel_two(self): #steer값 오른쪽이 -값 -0.45 ~ 0.48

        #time.time으로 변경
        current_time = time.time_ns()
        delta_time = (current_time-self.time_old)/10**9
        self.time_old = current_time
        if self.encoder_tic_left != None and self.encoder_tic_right != None:
            if self.j==0:
                self.old_enc_left=self.encoder_tic_left
                self.old_enc_right=self.encoder_tic_right

            else:
                current_enc_left =self.encoder_tic_left
                current_enc_right =self.encoder_tic_right

                delta_enc_left=current_enc_left-self.old_enc_left
                delta_enc_right=current_enc_right-self.old_enc_right

                if delta_enc_left > 2000000000:
                    delta_enc_left - 2147483647
                if delta_enc_left < -2000000000:
                    delta_enc_left + 2147483647
                self.old_enc_left=current_enc_left

                if delta_enc_right > 2000000000:
                    delta_enc_right - 2147483647
                if delta_enc_right < -2000000000:
                    delta_enc_right + 2147483647
                self.old_enc_right=current_enc_right

                delta_pos=  (delta_enc_left-delta_enc_right)/2 * 0.06283185307 * 0.26 /4 #4체배

                print(delta_pos,"yea")
                self.encoder_vel_two.twist.twist.linear.x= ( delta_pos / delta_time )

            self.j=1
        #if(self.encoder_vel0.twist.twist.linear.x != 0):
        self.pub_enc_vel_two.publish(self.encoder_vel_two)


    def TICK2RAD(self, tic):

      rad = tic*0.06283185307
      return rad

    def steer_callback(self, msg):
        input_steer = msg.drive.steering_angle

        self.steer = self.filter.update(input_steer)

    def pub_encoder_vel_one(self):
        if self.encoder_tic_left != None:

            current_time = time.time()
            delta_time=current_time-self.time_old3
            #print(delta_time)
            self.time_old3=current_time

            self.delta_encoder3 = self.encoder_tic_left - self.last_encoder3

            print("delta_enc_one",self.delta_encoder3)

            self.last_encoder3 = self.encoder_tic_left
            if (self.delta_encoder3 <= -2000000000):
                self.delta_encoder3 = self.delta_encoder3 + 2147483647
            elif (self.delta_encoder3 >= 2000000000):
                self.delta_encoder3 = self.delta_encoder3 - 2147483647

            #print("encoder:", self.encoder)

            #print("del end",self.delta_encoder)

            if(self.delta_encoder3 < 200):
                self.wheel_pos = self.TICK2RAD(self.delta_encoder3)
            else: return

            if(self.steer == 0):
                self.encoder_vel_one.twist.twist.linear.x = self.TICK2RAD(self.delta_encoder3) * 0.265 / delta_time
            else:

                L = self.wheel_base / abs(m.tan(self.steer))
                r_center = m.sqrt(pow(L, 2) + pow(self.wheel_base, 2))
                r_in = m.sqrt(pow((L - self.wheel_tread/2), 2) + pow(self.wheel_base, 2))
                r_out = m.sqrt(pow((L + self.wheel_tread/2), 2) + pow(self.wheel_base, 2))
                theta_center = self.steer
                theta_in = m.atan(self.wheel_tread / (L - self.wheel_tread/2))
                theta_out = m.atan(self.wheel_tread / (L + self.wheel_tread/2))

                #속도 8
                # speed_up = 1 - (1 - r_center / r_in)*0
                # speed_down = 1 - (1 - r_center / r_out)*0.2

                speed_up = 1 - (1 - r_center / r_in)
                speed_down = 1 - (1 - r_center / r_out)*1.5

            if self.steer > 0.4:
                self.delta_pos3 = self.wheel_radius * self.wheel_pos * speed_up /4
                self.get_logger().info('up vel gain: %f' %speed_up)
            elif self.steer < -0.4:
                self.delta_pos3 = self.wheel_radius * self.wheel_pos * speed_down /4
                self.get_logger().info('down vel gain: %f' %speed_down)
            else:
                self.delta_pos3 = self.wheel_radius * self.wheel_pos /4

            self.linear_vel3 = self.delta_pos3 / delta_time
            self.angular_vel3 = m.tan(self.steer) * self.linear_vel3 / self.wheel_base

            # self.get_logger().info('up vel gain: %f' % r_center / r_in)
            # self.get_logger().info('down vel gain: %f' % r_center / r_out)
            #self.vel_now = self.linear_vel


            if self.i3 == 0:
                self.old_yaw3 = 0
                self.before_yaw = self.yaw3

            """
            else:
                if abs(self.yaw - self.before_yaw) > 0.1:
                    self.before_yaw = self.yaw
                    self.yaw = self.yaw * 0.9
            """

            self.odom_yaw3 = self.old_yaw3 + self.yaw3
            self.odom_x3 -= self.delta_pos3 * m.cos(self.odom_yaw3)
            self.odom_y3 -= self.delta_pos3 * m.sin(self.odom_yaw3)

            #print("Duration:", delta_time)
            #print("Previous Encoder:", self.last_encoder)
            # print("Delta Encoder:", self.delta_encoder)

            self.i3 += 1

            self.encoder_vel_one.twist.twist.linear.x = self.linear_vel3

            #if(self.linear_vel != 0):
            if(self.linear_vel3 < 10 and self.linear_vel3 >-10 ):

                self.pub_enc_vel_one.publish(self.encoder_vel_one)

    def data_callback(self):
        self.pub_encoder_vel_one()
        self.pub_encoder_vel_two()


def main(args=None):
  rclpy.init(args=args)
  node = Pub_Two_Enc_Vel2()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt')
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
	main()
