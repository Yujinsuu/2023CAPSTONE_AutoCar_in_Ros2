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
import math
import numpy as np
import numpy as np

def euler_from_quaternion(x, y, z, w):

	"""
	Convert a quaternion into euler angles (roll, pitch, yaw)
	roll is rotation around x in radians (counterclockwise)
	pitch is rotation around y in radians (counterclockwise)
	yaw is rotation around z in radians (counterclockwise)
	"""

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	roll_x = math.atan2(t0, t1)

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch_y = math.asin(t2)

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw_z = math.atan2(t3, t4)

	return yaw_z

def get_quaternion_from_euler(roll, pitch, yaw):

	"""
	Convert an Euler angle to a quaternion.

	Input
	:param roll: The roll (rotation around x-axis) angle in radians.
	:param pitch: The pitch (rotation around y-axis) angle in radians.
	:param yaw: The yaw (rotation around z-axis) angle in radians.

	Output
	:return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
	"""

	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

	return [qx, qy, qz, qw]

class odomPublisher(Node):
	def __init__(self):
		super().__init__('odom_pub')
		self.gpose = Odometry()
		self.gpose.header.stamp = self.get_clock().now().to_msg()
		self.gpose.header.frame_id = 'odom'
		qos_profile = QoSProfile(depth=10)
		self.gps_sub = self.create_subscription(NavSatFix, '/ublox_gps/fix', self.gps_callback, qos_profile)
		self.gps_vel_sub = self.create_subscription(TwistWithCovarianceStamped, '/ublox_gps/fix_velocity', self.gps_vel_callback, qos_profile)
		self.imu_sub = self.create_subscription(QuaternionStamped, '/filter/quaternion', self.imu_callback, qos_profile)
		self.odom_pub = self.create_publisher(Odometry, '/autocar/odom', qos_profile)

		self.timer = self.create_timer(0.1, self.odom_publish)

	def gps_callback(self, gps):
		# cov1 = gps.position_covariance[0]
		# cov2 = gps.position_covariance[4]
		# cov3 = gps.position_covariance[8]
		transformer = Transformer.from_crs('EPSG:4326', 'EPSG:5178')
		a, b = transformer.transform(gps.latitude, gps.longitude)
		# p1 = Proj(init='epsg:4326')
		# p2 = Proj(init='epsg:5179')
		# a, b = transform(p1, p2, gps.longitude, gps.latitude)

		self.gpose.pose.pose.position.x=b-962897.516413939
		self.gpose.pose.pose.position.y=a-1958728.3104721
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
		imu_yaw = euler_from_quaternion(imu.quaternion.x, imu.quaternion.y, imu.quaternion.z, imu.quaternion.w)
		imu_yaw +=  np.deg2rad(0) # 오차 보정

		imu_qx, imu_qy, imu_qz, imu_qw = get_quaternion_from_euler(0,0,imu_yaw)

		self.gpose.pose.pose.orientation.x= imu_qx
		self.gpose.pose.pose.orientation.y= imu_qy
		self.gpose.pose.pose.orientation.z= imu_qz
		self.gpose.pose.pose.orientation.w= imu_qw

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
