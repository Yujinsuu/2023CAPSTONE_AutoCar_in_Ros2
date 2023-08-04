#! /usr/bin/env python3
#-*- coding: utf-8 -*-

import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from autocar_msgs.msg import State2D
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from autocar_nav.quaternion import yaw_to_quaternion, euler_from_quaternion


class odom_viz(Node):

  def __init__(self):

    super().__init__('odom_viz')

    self.marker_pub=self.create_publisher(Marker, "/rviz/odom_marker", 1)
    self.viz_steer = self.create_publisher(Marker, '/rviz/viz_steer', 1)
    self.viz_yaw = self.create_publisher(Marker, '/rviz/viz_yaw', 1)

    # self.odom_sub=self.create_subscription(Odometry, "/autocar/odom", self.odometry_callback, 10)
    self.state_sub=self.create_subscription(State2D, "/autocar/state2D", self.state_callback, 10)
    self.cmd_sub = self.create_subscription(AckermannDriveStamped, '/autocar/autocar_cmd', self.visual_steer, 10)

    self.x = 0.0
    self.y = 0.0
    self.v = 0.0
    self.yaw = 0.0
    self.L = 1.04 / 2

    self.quat = Quaternion()

  def odometry_callback(self, data):
		# sensor_msgs/Imu.msg
    self.x = data.pose.pose.position.x
    self.y = data.pose.pose.position.y
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    self.v = np.sqrt(vx**2+vy**2)

    self.yaw = euler_from_quaternion(
      data.pose.pose.orientation.x,
      data.pose.pose.orientation.y,
      data.pose.pose.orientation.z,
      data.pose.pose.orientation.w
      )

    self.x = self.x + self.L * math.cos(self.yaw)
    self.y = self.y + self.L * math.sin(self.yaw)

    self.msg_pub()
    self.visual_yaw()

  def state_callback(self, state):
    self.x = state.pose.x
    self.y = state.pose.y
    vx = state.twist.x
    vy = state.twist.y
    self.v = np.sqrt(vx**2+vy**2)

    self.yaw = state.pose.theta

    self.x = self.x + self.L * math.cos(self.yaw)
    self.y = self.y + self.L * math.sin(self.yaw)

    self.msg_pub()
    self.visual_yaw()


  def msg_pub(self):
    self.quat = yaw_to_quaternion(self.yaw)

    m = Marker()
    m.header.frame_id = "odom"
    m.header.stamp = self.get_clock().now().to_msg()
    m.id = 1
    m.type = m.CUBE
    m.pose.position.x = self.x
    m.pose.position.y = self.y
    m.pose.position.z = 0.45

    m.pose.orientation.x = self.quat.x
    m.pose.orientation.y = self.quat.y
    m.pose.orientation.z = self.quat.z
    m.pose.orientation.w = self.quat.w

    m.scale.x = 1.600
    m.scale.y = 1.160
    m.scale.z = 1.000
    m.color.r = 17 / 255.0
    m.color.g = 17 / 255.0
    m.color.b = 252 / 255.0
    m.color.a = 0.97
    self.marker_pub.publish(m)

  def visual_yaw(self):
    marker = Marker()
    marker.header.frame_id = 'odom'
    marker.header.stamp = self.get_clock().now().to_msg()
    marker.id = 1
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = self.x
    marker.pose.position.y = self.y
    marker.pose.position.z = 0.3
    marker.pose.orientation.x = self.quat.x
    marker.pose.orientation.y = self.quat.y
    marker.pose.orientation.z = self.quat.z
    marker.pose.orientation.w = self.quat.w
    marker.scale.x = 2.
    marker.scale.y = 0.3
    marker.scale.z = 0.1
    marker.color.r = 17 / 255.0
    marker.color.g = 17 / 255.0
    marker.color.b = 252 / 255.0
    marker.color.a = 0.97
    self.viz_yaw.publish(marker)

  def visual_steer(self, msg):
    steer = msg.drive.steering_angle
    theta = self.yaw + steer

    marker = Marker()
    marker.header.frame_id = 'odom'
    marker.header.stamp = self.get_clock().now().to_msg()
    marker.ns = 'arrows'
    marker.id = 0
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
