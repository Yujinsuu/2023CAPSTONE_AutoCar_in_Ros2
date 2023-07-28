#! /usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from autocar_msgs.msg import State2D
from geometry_msgs.msg import TransformStamped

from tf2_ros import StaticTransformBroadcaster

class TF_Publisher(Node):
  def __init__(self):
    super().__init__('tf_pub')

    self.localisation_sub = self.create_subscription(State2D, '/autocar/state2D', self.vehicle_state_cb, 10)

    self.x = 0.0
    self.y = 0.0
    self.yaw = 0.0

    self.GtoL = 1.29 # gps to lidar distance

    self.tf_broadcaster = StaticTransformBroadcaster(self)

  def vehicle_state_cb(self, msg):
    self.x = msg.pose.x
    self.y = msg.pose.y
    self.yaw = msg.pose.theta

    # create car frame
    transform = TransformStamped()
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'car'
    transform.transform.translation.x = self.x + self.GtoL * np.cos(self.yaw)
    transform.transform.translation.y = self.y + self.GtoL * np.sin(self.yaw)
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = np.sin(self.yaw / 2)
    transform.transform.rotation.w = np.cos(self.yaw / 2)

    # Broadcast the transform as a static transform
    self.tf_broadcaster.sendTransform(transform)


def main(args=None):
  rclpy.init(args=args)
  node = TF_Publisher()

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
