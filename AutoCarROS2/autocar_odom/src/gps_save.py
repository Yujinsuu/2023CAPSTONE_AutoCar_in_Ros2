#! /usr/bin/env python3

import os
import pandas as pd
from pyproj import Transformer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import NavSatFix

from autocar_nav.normalise_angle import normalise_angle
from autocar_nav.quaternion import yaw_to_quaternion, euler_from_quaternion


class odomPublisher(Node):

	def __init__(self):

		super().__init__('odom_pub')
		qos_profile = QoSProfile(depth=10)
		self.gps_sub = self.create_subscription(NavSatFix, '/ublox_gps/fix', self.gps_callback, qos_profile)

		self.gps_offset = {'seoul':[962897.516413939,1958728.3104721],'kcity':[935504.1834692371,1915769.1316598575]}

		self.x = []
		self.y = []

	def gps_callback(self, gps):

		transformer = Transformer.from_crs('EPSG:4326', 'EPSG:5179')
		a, b = transformer.transform(gps.latitude, gps.longitude)

		self.x.append(b)
		self.y.append(a)

	def save(self, output_folder):
		count = 0
		output = os.path.join(output_folder, f'gps_{count}.csv')
		while os.path.exists(output):
				count += 1
				output = os.path.join(output_folder, f'gps_{count}.csv')

		df = pd.DataFrame({'X':self.x,'Y':self.y})
		df.to_csv(output, index=False, mode='w', encoding='utf-8-sig')


def main(args=None):
	rclpy.init(args=args)
	node = odomPublisher()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		home_folder = os.path.expanduser("~")
		dataset_folder = os.path.join(home_folder, 'dataset', 'gps_data')
		node.save(dataset_folder)
		node.get_logger().info('GPS Data Saved')
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__=='__main__':
	main()
