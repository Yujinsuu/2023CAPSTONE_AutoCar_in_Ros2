#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped

from tf2_ros import StaticTransformBroadcaster

class TF_Publisher(Node):
	def __init__(self):
		super().__init__('tf_pub')

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
