#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, QuaternionStamped
from nav_msgs.msg import Odometry
from pyproj import Proj, transform, CRS, Transformer
from rclpy.clock import Clock
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

import numpy as np

class odomPublisher(Node):
	def __init__(self):
		super().__init__('odom_pub')
		self.gpose = Odometry()
		self.gpose.header.stamp = self.get_clock().now().to_msg()
		self.gpose.header.frame_id = 'odom'
		qos_profile = QoSProfile(depth=10)
		self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, qos_profile)
		self.gps_vel_sub = self.create_subscription(TwistWithCovarianceStamped, '/fix_velocity', self.gps_vel_callback, qos_profile)
		self.imu_sub = self.create_subscription(QuaternionStamped, '/filter/quaternion', self.imu_callback, qos_profile)
		self.odom_pub = self.create_publisher(Odometry, '/autocar/odom', qos_profile)

		self.timer = self.create_timer(0.1, self.odom_publish)

	def gps_callback(self, gps):
		# cov1 = gps.position_covariance[0]
		# cov2 = gps.position_covariance[4]
		# cov3 = gps.position_covariance[8]
		transformer = Transformer.from_crs('EPSG:4326', 'EPSG:5179')
		a, b = transformer.transform(gps.latitude, gps.longitude)
		# p1 = Proj(init='epsg:4326')
		# p2 = Proj(init='epsg:5179')
		# a, b = transform(p1, p2, gps.longitude, gps.latitude)

		self.gpose.pose.pose.position.x=b #-1.8177898578578606
		self.gpose.pose.pose.position.y=a #+0.34575470979325473
		# self.gpose.pose.covariance[0]=cov1
		# self.gpose.pose.covariance[7]=cov2
		# self.gpose.pose.covariance[14]=cov3
		# self.gpose.pose.covariance[21]=cov3
		# self.gpose.pose.covariance[28]=cov3
		# self.gpose.pose.covariance[35]=cov3

	def gps_vel_callback(self, gps_vel):
		self.gpose.twist.twist.linear.x = gps_vel.twist.twist.linear.x
		self.gpose.twist.twist.linear.y = gps_vel.twist.twist.linear.y
		self.gpose.twist.twist.linear.z = gps_vel.twist.twist.linear.z


	def imu_callback(self, imu):
		self.gpose.pose.pose.orientation.x= imu.quaternion.x
		self.gpose.pose.pose.orientation.y= imu.quaternion.y
		self.gpose.pose.pose.orientation.z= imu.quaternion.z
		self.gpose.pose.pose.orientation.w= imu.quaternion.w

	def odom_publish(self):
		self.odom_pub.publish(self.gpose)

def main(args=None):
  rclpy.init(args=args)
  node = odomPublisher()

  # create odom frame
  transform = TransformStamped()
  transform.header.frame_id = 'map'
  transform.child_frame_id = 'odom'

  # Broadcast the transform as a static transform
  static_broadcaster = StaticTransformBroadcaster(node)
  static_broadcaster.sendTransform(transform)

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt')
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
	main()
