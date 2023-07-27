#! /usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from autocar_msgs.msg import LinkArray


class Dead_Reckoning(Node):
		def __init__(self):
				super().__init__('dead_reckoning')

				qos_profile = QoSProfile(depth=10)
				self.GPS_odom_sub = self.create_subscription(Odometry, '/data/gps', self.raw_data_callback, qos_profile)
				self.EKF_odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.filtered_data_callback, qos_profile)
				self.mode_sub = self.create_subscription(LinkArray, '/autocar/mode', self.mode_cb, 10)

				self.dr_odom_pub = self.create_publisher(Odometry, '/dr_odom', qos_profile)

				self.mode = 'global'
				self.mode_change = 0

				self.raw_x = 0.0
				self.raw_y = 0.0
				self.gps_vel = 0.0

				self.offset_x = 0.0
				self.offset_y = 0.0
				self.init_x = 0.0
				self.init_y = 0.0

				self.dr_odom = None
				self.dr_x = None
				self.dr_y = None
				self.dr_yaw = None


		def mode_cb(self, msg):
				self.mode = msg.mode
				if self.mode == 'tunnel':
						self.mode_change += 1


		def raw_data_callback(self, data):
				#get gps position
				self.raw_x = data.pose.pose.position.x
				self.raw_y = data.pose.pose.position.y
				xvel = data.twist.twist.linear.x
				yvel = data.twist.twist.linear.y

				self.gps_vel = math.sqrt(pow(xvel,2)+pow(yvel,2))


		def filtered_data_callback(self, data):
				if self.mode_change == 1:
						self.offset_x = self.raw_x
						self.offset_y = self.raw_y
						self.init_x = data.pose.pose.position.x
						self.init_y = data.pose.pose.position.y

				self.dr_odom = Odometry()
				self.dr_odom = data

				#offset filterd position
				self.dr_odom.pose.pose.position.x = data.pose.pose.position.x - self.init_x
				self.dr_odom.pose.pose.position.y = data.pose.pose.position.y - self.init_y

				self.dr_x = self.dr_odom.pose.pose.position.x
				self.dr_y = self.dr_odom.pose.pose.position.y
				self.dr_yaw = 2.0 * np.arctan2(self.dr_odom.pose.pose.orientation.z, self.dr_odom.pose.pose.orientation.w)

				#publish final odom
				self.dr_odom_pub.publish(self.dr_odom)


def main(args=None):
  rclpy.init(args=args)
  node = Dead_Reckoning()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt')
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
	main()
