#! /usr/bin/env python3

import time
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Int32


class Pub_Encoder_Tic(Node):
    def __init__(self):
        super().__init__('pub_encoder_tic')

        qos_profile = QoSProfile(depth=10)

        self.pub_enc_tic = self.create_publisher(Int32, '/data/encoder_tic', qos_profile)

        self.ser = serial.serial_for_url("/dev/ttyARDUINO", baudrate=9600, timeout=0.01)

        self.encoder_tic = Int32()

    def get_value(self):

        s = self.ser.readline()
        #print(s)
        try:
            self.encoder_tic.data = int(s.decode().strip())
            self.pub_enc_tic.publish(self.encoder_tic)
            print(self.encoder_tic.data,'/', time.time_ns()/10**9)
        except:
            pass


def main(args=None):
  rclpy.init(args=args)
  node = Pub_Encoder_Tic()

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
