#! /usr/bin/env python3

import time
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Int32MultiArray


class Pub_Two_Encoder_Tic(Node):
    def __init__(self):
        super().__init__('pub_enc_tic')

        self.encoder_tic = Int32MultiArray()
        qos_profile = QoSProfile(depth=10)

        self.pub_enc_tic = self.create_publisher(Int32MultiArray, '/data/encoder_tic', qos_profile)
        self.ser = serial.serial_for_url("/dev/ttyARDUINO", baudrate=9600, timeout=0.01)

    def get_value(self):

        s = self.ser.readline()
        #print(s)

        try:
            decode_encoder = s.decode().strip()
            #print(decode_encoder)
            dec_enc = decode_encoder.split(",")
            self.encoder_tic.data = [int(dec_enc[0]),int(dec_enc[1])]
            print(type(self.encoder_tic.data))
            self.pub_enc_tic.publish(self.encoder_tic)
            #print("left: ", self.encoder_tic.data[0])
            #print("right: ", self.encoder_tic.data[1])
        except:
            pass


def main(args=None):
  rclpy.init(args=args)
  node = Pub_Two_Encoder_Tic()

  try:
    while True:
      node.get_value()

  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt')
  finally:
    node.destroy_node()
    rclpy.shutdown()



if __name__=='__main__':
	main()
