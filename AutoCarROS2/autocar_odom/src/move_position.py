#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import time
import getch
import threading

class Pub_Position_Offset(Node):
    def __init__(self):
        super().__init__('pub_position_offset')

        self.get_logger().info(" 'g': gps mode, 'd': dead reckoning mode, 'q': quit")
        self.get_logger().info("8,2 y값증감, 6,4 x값 증감, 5 초기화")
        self.key_listener_thread = threading.Thread(target=self.listen_for_key)
        self.key_listener_thread.start()

        self.count = 0
        self.quit = False

        self.gps_mode = False
        self.dr_mode = False

        self.position_offset_msg = Float32MultiArray() # gx gy dx dy
        self.position_offset_msg.data = [0.0,0.0,0.0,0.0]
        self.gx = 0.0
        self.gy = 0.0
        self.dx = 0.0
        self.dy = 0.0

        self.pose_offset_pub = self.create_publisher(Float32MultiArray, '/data/key_offset', 10)



    def listen_for_key(self):
        while rclpy.ok():
            key = getch.getch()
            # print(key)
            #self.get_logger().info(key)

            if key == 'g':
                self.get_logger().info("gps mode")
                self.gps_mode = True
                self.dr_mode = False

            if key == 'd':
                self.get_logger().info("dr mode")
                self.dr_mode = True
                self.gps_mode = False

            if self.gps_mode:
                if key == '8':
                    self.gx += 0.3

                elif key == '2':
                    self.gx -= 0.3

                elif key == '4':
                    self.gy += 0.3

                elif key == '6':
                    self.gy -= 0.3

                elif key == '5':
                    self.gx = 0.0
                    self.gy = 0.0



            if self.dr_mode:
                if key == '8':
                    self.dx += 0.3

                elif key == '2':
                    self.dx -= 0.3

                elif key == '4':
                    self.dy += 0.3

                elif key == '6':
                    self.dy -= 0.3

                elif key == '5':
                    self.dx = 0.0
                    self.dy = 0.0
            self.position_offset_msg.data[0] = self.gx
            self.position_offset_msg.data[1] = self.gy
            self.position_offset_msg.data[2] = self.dx
            self.position_offset_msg.data[3] = self.dy

            print('----------------')
            print("gx: ", self.gx)
            print("gy: ", self.gy)
            print("dx: ", self.dx)
            print("dy: ", self.dy)


            self.pose_offset_pub.publish(self.position_offset_msg)

            if key == 'a':
                self.count += 1
                self.get_logger().info(f'Count increased: {self.count}')



            if key == 'q':
                self.destroy_node()
                rclpy.shutdown()
                self.quit == True
                break

def main(args=None):
  rclpy.init(args=args)
  node = Pub_Position_Offset()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt')


if __name__=='__main__':
	main()