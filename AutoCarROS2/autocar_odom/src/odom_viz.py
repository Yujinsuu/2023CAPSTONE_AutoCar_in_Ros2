#! /usr/bin/env python3
#-*- coding: utf-8 -*-

import math
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from autocar_msgs.msg import State2D
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray

from autocar_nav.quaternion import yaw_to_quaternion, euler_from_quaternion


class odom_viz(Node):

  def __init__(self):

    super().__init__('odom_viz')

    self.GPS_Car_pub = self.create_publisher(MarkerArray, "/rviz/gps_odometry_marker", 1)
    self.DR_Car_pub = self.create_publisher(MarkerArray, "/rviz/dead_reckoning_marker", 1)
    self.viz_steer = self.create_publisher(Marker, '/rviz/viz_steer', 1)

    self.GPS_Odom_sub = self.create_subscription(Odometry, "/autocar/odom", self.gps_odometry_callback, 10)
    self.DR_Odom_sub = self.create_subscription(Odometry, "/odometry/filtered", self.dead_reckoning_callback, 10)
    self.DR_offset_sub = self.create_subscription(Float64MultiArray, '/autocar/dr_offset', self.offset_callback, 10)
    self.state_sub = self.create_subscription(State2D, "/autocar/state2D", self.state_callback, 10)
    self.cmd_sub = self.create_subscription(AckermannDriveStamped, '/autocar/autocar_cmd', self.visual_steer, 10)

    self.L = 1.04 / 2

    self.x = None
    self.Gx = None
    self.Dx = None

    self.offset_x = 0.0
    self.offset_y = 0.0


  def state_callback(self, state):
    self.x = state.pose.x
    self.y = state.pose.y
    self.yaw = state.pose.theta

    self.x = self.x + self.L * math.cos(self.yaw)
    self.y = self.y + self.L * math.sin(self.yaw)


  def gps_odometry_callback(self, data):
		# sensor_msgs/Imu.msg
    self.Gx = data.pose.pose.position.x
    self.Gy = data.pose.pose.position.y
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    self.Gv = np.sqrt(vx**2+vy**2)

    self.Gyaw = euler_from_quaternion(
      data.pose.pose.orientation.x,
      data.pose.pose.orientation.y,
      data.pose.pose.orientation.z,
      data.pose.pose.orientation.w
      )

    self.Gx = self.Gx + self.L * math.cos(self.Gyaw)
    self.Gy = self.Gy + self.L * math.sin(self.Gyaw)

    car = MarkerArray()
    m1 = self.marker_pub(self.Gx, self.Gy, self.Gyaw, 'GPS')
    car.markers.append(m1)
    m2 = self.visual_yaw(self.Gx, self.Gy, self.Gyaw, 'GPS')
    car.markers.append(m2)

    self.GPS_Car_pub.publish(car)

  def offset_callback(self, msg):
    self.offset_x = msg.data[0]
    self.offset_y = msg.data[1]

  def dead_reckoning_callback(self, data):
		# sensor_msgs/Imu.msg
    self.Dx = data.pose.pose.position.x + self.offset_x
    self.Dy = data.pose.pose.position.y + self.offset_y
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    self.Dv = np.sqrt(vx**2+vy**2)

    self.Dyaw = euler_from_quaternion(
      data.pose.pose.orientation.x,
      data.pose.pose.orientation.y,
      data.pose.pose.orientation.z,
      data.pose.pose.orientation.w
      )

    self.Dx = self.Dx + self.L * math.cos(self.Dyaw)
    self.Dy = self.Dy + self.L * math.sin(self.Dyaw)

    car = MarkerArray()
    m1 = self.marker_pub(self.Dx, self.Dy, self.Dyaw, 'DR')
    car.markers.append(m1)
    m2 = self.visual_yaw(self.Dx, self.Dy, self.Dyaw, 'DR')
    car.markers.append(m2)

    self.DR_Car_pub.publish(car)


  def marker_pub(self, x, y, yaw, state):
    quat = yaw_to_quaternion(yaw)

    m = Marker()
    m.header.frame_id = "odom"
    m.header.stamp = self.get_clock().now().to_msg()

    m.id = 1
    m.type = m.CUBE
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.45

    m.pose.orientation.x = quat.x
    m.pose.orientation.y = quat.y
    m.pose.orientation.z = quat.z
    m.pose.orientation.w = quat.w

    m.scale.x = 1.600
    m.scale.y = 1.160
    m.scale.z = 1.000

    if state == 'GPS':
      m.color.r = 17 / 255.0
      m.color.g = 17 / 255.0
      m.color.b = 252 / 255.0
    else:
      m.color.r = 17 / 255.0
      m.color.g = 252 / 255.0
      m.color.b = 17 / 255.0
    m.color.a = 0.97

    return m

  def visual_yaw(self, x, y, yaw, state):
    quat = yaw_to_quaternion(yaw)

    m = Marker()
    m.header.frame_id = 'odom'
    m.header.stamp = self.get_clock().now().to_msg()

    m.id = 2
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.3

    m.pose.orientation.x = quat.x
    m.pose.orientation.y = quat.y
    m.pose.orientation.z = quat.z
    m.pose.orientation.w = quat.w

    m.scale.x = 2.
    m.scale.y = 0.3
    m.scale.z = 0.1

    if state == 'GPS':
      m.color.r = 17 / 255.0
      m.color.g = 17 / 255.0
      m.color.b = 252 / 255.0
    else:
      m.color.r = 17 / 255.0
      m.color.g = 252 / 255.0
      m.color.b = 17 / 255.0
    m.color.a = 0.97

    return m

  def visual_steer(self, msg):
    marker = Marker()
    if self.x is not None:
      steer = msg.drive.steering_angle
      theta = self.yaw + steer

      marker.header.frame_id = 'odom'
      marker.header.stamp = self.get_clock().now().to_msg()

      marker.ns = 'arrows'
      marker.id = 1
      marker.type = Marker.ARROW
      marker.action = Marker.ADD

      marker.pose.position.x = self.x
      marker.pose.position.y = self.y
      marker.pose.position.z = 0.6

      marker.pose.orientation.x = 0.0
      marker.pose.orientation.y = 0.0
      marker.pose.orientation.z = np.sin(theta / 2)
      marker.pose.orientation.w = np.cos(theta / 2)

      marker.scale.x = 2.
      marker.scale.y = 0.3
      marker.scale.z = 0.1

      marker.color.r = 1.0
      marker.color.g = 0.0
      marker.color.b = 0.0
      marker.color.a = 1.0

    self.viz_steer.publish(marker)

def main(args=None):
  rclpy.init(args=args)
  node = odom_viz()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt')
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()
