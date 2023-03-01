#! /usr/bin/env python3
#-*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import math


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
  return roll_x, pitch_y, yaw_z # in radians

def quaternion_from_euler(roll, pitch, yaw):
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


class odom_viz(Node):

  def __init__(self):
    super().__init__('odom_viz')
    self.marker_pub=self.create_publisher(PoseStamped, "/rviz/autocar", 1)
    self.odom_sub=self.create_subscription(Odometry, "/autocar/odom", self.odometry_callback, 10)
    #self.tf_broadcaster = tf2_ros.TransformBroadcaster()

  def odometry_callback(self, data):
    # sensor_msgs/Imu.msg
    self.x = data.pose.pose.position.x
    self.y = data.pose.pose.position.y
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    self.v = np.sqrt(vx**2+vy**2)

    orientation_list = [
      data.pose.pose.orientation.x,
      data.pose.pose.orientation.y,
      data.pose.pose.orientation.z,
      data.pose.pose.orientation.w
      ]

    roll, pitch, self.yaw = euler_from_quaternion(
      orientation_list[0],
      orientation_list[1],
      orientation_list[2],
      orientation_list[3]
      )

    self.yaw+=0.1
    self.msg_pub()


  def msg_pub(self):

    c = PoseStamped()
    c.header.stamp = self.get_clock().now().to_msg()
    c.header.frame_id = 'odom'

    c.pose.position.x = self.x
    c.pose.position.y = self.y
    c.pose.position.z = 1.0

    quat = quaternion_from_euler(0, 0, self.yaw)
    c.pose.orientation.x = quat[0]
    c.pose.orientation.y = quat[1]
    c.pose.orientation.z = quat[2]
    c.pose.orientation.w = quat[3]

    self.marker_pub.publish(c)

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
