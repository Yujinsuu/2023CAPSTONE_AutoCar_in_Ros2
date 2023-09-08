#! /usr/bin/env python3

import math
import numpy as np
from pyproj import Transformer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, Imu
from autocar_msgs.msg import LinkArray
from geometry_msgs.msg import TwistWithCovarianceStamped, QuaternionStamped, Vector3Stamped, TransformStamped

from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from autocar_nav.normalise_angle import normalise_angle
from autocar_nav.quaternion import yaw_to_quaternion, euler_from_quaternion

class odomPublisher(Node):

	def __init__(self):

		super().__init__('odom_pub')
		qos_profile = QoSProfile(depth=10)
		self.odom_pub = self.create_publisher(Odometry, '/autocar/odom', qos_profile)
		self.data_pub_gps = self.create_publisher(Odometry, '/data/gps', qos_profile)
		self.data_pub_imu = self.create_publisher(Imu, '/data/imu', qos_profile)
		self.pub_yaw_offset_av = self.create_publisher(Float32, 'yaw_offset_av', qos_profile)


		self.gps_sub = self.create_subscription(NavSatFix, '/ublox_gps/fix', self.gps_callback, qos_profile)
		self.gps_vel_sub = self.create_subscription(TwistWithCovarianceStamped, '/ublox_gps/fix_velocity', self.gps_vel_callback, qos_profile)
		self.imu_sub = self.create_subscription(QuaternionStamped, '/filter/quaternion', self.imu_callback, qos_profile)
		self.imu_angularV_sub = self.create_subscription(Vector3Stamped, '/imu/angular_velocity', self.imu_angularV_callback, qos_profile)
		self.encoder_sub = self.create_subscription(Odometry, '/data/encoder_vel', self.encoder_callback, 10)
		self.mode_sub = self.create_subscription(LinkArray,'/autocar/mode', self.mode_callback, qos_profile )

		self.heading_array = []
		self.heading = 0
		self.filtered_heading = 0
		self.i=0
		self.before_qz = 0
		self.before_qw = 1

		self.gps_yaw = 0.0
		self.imu_yaw = 0.0
		self.velocity = 0.0
		self.gps_offset = {'seoul':[962897.516413939,1958728.3104721],'kcity':[935504.1834692371,1915769.1316598575]}
		self.yaw_offset = 0.0
		self.final_imu_yaw = 0.0
		self.set_odom_tf = 0
		self.yaw_offset_array = []
		self.yaw_offset_av = 0.0
		self.corr = None
		self.encoder_vel = 0.0
		self.yaw_offset_av_print = 0.0
		self.yaw_offset_av_pub = Float32()
		self.last_corr = False

		self.x_cov = 0.0
		self.y_cov = 0.0
		self.corr_mode = False

		self.gpose = Odometry()
		self.gpose.header.stamp = self.get_clock().now().to_msg()
		self.gpose.header.frame_id = 'odom'
		self.gps_data = Odometry()
		self.gps_data.header.stamp = self.get_clock().now().to_msg()
		self.gps_data.header.frame_id = 'odom'
		self.imu_data = Imu()
		self.imu_data.header.stamp = self.get_clock().now().to_msg()
		self.imu_data.header.frame_id = 'odom_footprint'


		self.declare_parameter('yaw_init', 167)
		self.declare_parameter('yaw_offset_array', [])
		self.yaw_init = self.get_parameter('yaw_init').value
		self.yaw_offset_array = self.get_parameter('yaw_offset_array').value
		self.add_on_set_parameters_callback(self.update_parameter)

		#self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', qos_profile)


		self.timer = self.create_timer(0.1, self.odom_publish)

		# self.gps_tf_publisher_static()

	def update_parameter(self, params):
		for param in params:
			if param.name == 'yaw_init' and (param.type_ == Parameter.Type.DOUBLE or param.type_ == Parameter.Type.INTEGER):
				self.yaw_init = param.value

			if param.name == 'yaw_offset_array':
				self.yaw_offset_array.clear()

		return SetParametersResult(successful=True)


	def encoder_callback(self, enc):
		self.encoder_vel = enc.twist.twist.linear.x

	def gps_callback(self, gps):

		transformer = Transformer.from_crs('EPSG:4326', 'EPSG:5179')
		a, b = transformer.transform(gps.latitude, gps.longitude)

		x = b - self.gps_offset['kcity'][0]-.5
		y = a - self.gps_offset['kcity'][1]

		self.gpose.pose.pose.position.x=x
		self.gpose.pose.pose.position.y=y

		#추가
		self.x_cov =  gps.position_covariance[0]
		self.y_cov = gps.position_covariance[4]
		self.gpose.pose.covariance[0] = self.x_cov
		self.gpose.pose.covariance[7] = self.y_cov
		if(self.x_cov < 0.01 and self.y_cov < 0.01):
			self.corr_mode = True
		else:
			self.corr_mode = False

		# self.set_odom_tf = self.set_odom_tf + 1
		# if(self.set_odom_tf == 1):
		# 	self.odom_tf_publisher_static()


		#publish for dead reckoning
		self.gps_data.pose.pose.position.x = x
		self.gps_data.pose.pose.position.y = y
		self.data_pub_gps.publish(self.gps_data)


	def gps_vel_callback(self, gps_vel):

		self.gpose.twist.twist.linear.x = gps_vel.twist.twist.linear.x
		self.gpose.twist.twist.linear.y = gps_vel.twist.twist.linear.y
		self.gpose.twist.twist.linear.z = gps_vel.twist.twist.linear.z

		self.heading=math.atan2(gps_vel.twist.twist.linear.y , gps_vel.twist.twist.linear.x)
		self.heading_array.insert(0,self.heading)

		if len(self.heading_array) == 3:   # save number
			self.heading_array.pop()
			self.filtered_heading = (sum(self.heading_array)/len(self.heading_array))   # moving average

		if self.i==0:
			self.before_qz = 0
			self.before_qw = 1

		if (abs(gps_vel.twist.twist.linear.x) or abs(gps_vel.twist.twist.linear.y)) > 0.1:
			now_qz=np.sin(self.filtered_heading / 2)
			now_qw=np.cos(self.filtered_heading / 2)

			gps_qz = now_qz
			gps_qw = now_qw

			self.before_qz = now_qz
			self.before_qw = now_qw

		else:
			gps_qz = self.before_qz
			gps_qw = self.before_qw

		self.velocity = np.sqrt(gps_vel.twist.twist.linear.x ** 2 + gps_vel.twist.twist.linear.y ** 2)

		if self.velocity > 7/3.6:
			self.gps_yaw = euler_from_quaternion(0.0, 0.0, gps_qz, gps_qw)
			self.yaw_offset = normalise_angle(self.final_imu_yaw - self.gps_yaw)

		self.i = self.i + 1

		# save gps vel
		self.gps_data.twist.twist.linear.x = gps_vel.twist.twist.linear.x
		self.gps_data.twist.twist.linear.y = gps_vel.twist.twist.linear.y


	def imu_angularV_callback(self, imuV):
		self.imu_data.angular_velocity.z = imuV.vector.z

	def mode_callback(self, msg):
		if not self.last_corr:
			if msg.mode == 'uturn':
				if self.corr == True:
					self.yaw_offset_av = sum(self.yaw_offset_array)/len(self.yaw_offset_array)
					self.yaw_init -= np.rad2deg(self.yaw_offset_av)
				self.corr = False
				self.yaw_offset_array.clear()

			# apply average yaw_offset when Straigt > Curve
			elif msg.direction == 'Straight' and self.velocity > 7/3.6:
				if self.corr_mode or msg.mode != 'tunnel' :
					self.yaw_offset_array.append(self.yaw_offset)
				if len(self.yaw_offset_array) != 0:
					self.yaw_offset_av_print = sum(self.yaw_offset_array)/len(self.yaw_offset_array)
				if len(self.yaw_offset_array) > 20:
					self.corr = True

			elif msg.direction == 'Curve':
				if self.corr == True:
					self.yaw_offset_av = sum(self.yaw_offset_array)/len(self.yaw_offset_array)
					self.yaw_init -= np.rad2deg(self.yaw_offset_av)
				self.corr = False
				self.yaw_offset_array.clear()

		if msg.mode == 'uturn':
			self.last_corr = True


	def imu_callback(self, imu):

		imu_yaw = euler_from_quaternion(imu.quaternion.x, imu.quaternion.y, imu.quaternion.z, imu.quaternion.w)
		self.imu_yaw = imu_yaw + np.deg2rad(self.yaw_init) # 오차 보정 #73
		self.get_logger().info(f'yaw_offset : {round(np.rad2deg(-self.yaw_offset),2)}\t offset_av : {round(np.rad2deg(-self.yaw_offset_av),2)}\t yaw_init : {round(self.yaw_init,2)}\t yaw_offset_av_realtime : {round(np.rad2deg(self.yaw_offset_av_print),2)}' )
		# self.get_logger().info(f'yaw_offset_array : {self.yaw_offset_array}')
		# self.get_logger().info('corr_mode: %s' % self.corr_mode)

		self.final_imu_yaw = normalise_angle(self.imu_yaw) #normalise_angle(self.imu_yaw - self.yaw_offset_av)
		imu_quat = yaw_to_quaternion(self.final_imu_yaw)
		self.gpose.pose.pose.orientation.x= imu_quat.x
		self.gpose.pose.pose.orientation.y= imu_quat.y
		self.gpose.pose.pose.orientation.z= imu_quat.z
		self.gpose.pose.pose.orientation.w= imu_quat.w

		#pub imu for EKF
		self.imu_data.orientation.x= imu_quat.x
		self.imu_data.orientation.y= imu_quat.y
		self.imu_data.orientation.z= imu_quat.z
		self.imu_data.orientation.w= imu_quat.w
		self.data_pub_imu.publish(self.imu_data)


	def odom_publish(self):

		# self.get_logger().info(f'GPS_vel : {round(self.velocity*3.6, 1)} km/h\t ENC_vel : {round(self.encoder_vel*3.6, 1)} km/h')
		# self.get_logger().info(f'GPS_vel : {round(self.velocity*3.6, 1)} km/h\t ENC_vel : {round(self.encoder_vel*3.6, 1)} km/h')
		# self.get_logger().info('gps yaw : %f' % self.gps_yaw)
		# self.get_logger().info('gps yaw : %f' % self.gps_yaw)
		# self.get_logger().info('imu yaw : %f' % self.imu_yaw)
		# self.get_logger().info('imu yaw : %f' % self.imu_yaw)
		#self.get_logger().info('yaw_offset_av: %s' % self.yaw_offset_array)
		#self.get_logger().info('yaw_offset_av: %s' % self.yaw_offset_array)
		#self.get_logger().info(f'yaw_offset : {round(np.rad2deg(-self.yaw_offset),2)}\t offset_av : {round(np.rad2deg(-self.yaw_offset_av),2)}\t yaw_init : {round(self.yaw_init,2)}')
		self.odom_pub.publish(self.gpose)
		self.odom_pub.publish(self.gpose)
		self.yaw_offset_av_pub.data = self.yaw_offset_av_print
		self.pub_yaw_offset_av.publish(self.yaw_offset_av_pub)


	# def gps_tf_publisher_static(self):

	# 	transform = TransformStamped()
	# 	transform.header.frame_id = 'map'
	# 	transform.child_frame_id = 'odom0'
	# 	transform.transform.translation.x = 0.0 #self.gpose.pose.pose.position.x
	# 	transform.transform.translation.y = 0.0 #self.gpose.pose.pose.position.y
	# 	# Broadcast the transform as a static transform
	# 	static_broadcaster = StaticTransformBroadcaster(self)
	# 	static_broadcaster.sendTransform(transform)

	# def odom_tf_publisher_static(self):
	# 	  # create odom frame
	# 	transform = TransformStamped()
	# 	transform.header.frame_id = 'map'
	# 	transform.child_frame_id = 'odom'
	# 	transform.transform.translation.x = 0.0 #self.gpose.pose.pose.position.x
	# 	transform.transform.translation.y = 0.0 #self.gpose.pose.pose.position.y
	# 	# Broadcast the transform as a static transform
	# 	static_broadcaster = StaticTransformBroadcaster(self)
	# 	static_broadcaster.sendTransform(transform)

	# def tf_publisher_dynamic(self):

	# 	 # create odom_footprint
	# 	transform = TransformStamped()
	# 	transform.header.stamp = self.get_clock().now().to_msg()
	# 	transform.header.frame_id = 'odom'
	# 	transform.child_frame_id = 'odom_footprint'
	# 	dynamic_broadcaster = TransformBroadcaster(self)
	# 	dynamic_broadcaster.sendTransform(transform)


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
