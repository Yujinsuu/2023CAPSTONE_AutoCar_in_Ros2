#! /usr/bin/env python3

import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class GPS_Save(Node):

   def __init__(self):

      super().__init__('save')
      self.save_pub = self.create_publisher(Bool, '/save', 10)

      self.check = Bool()

      self.timer = self.create_timer(0.1, self.save)

   async def get_input(self):
      loop = asyncio.get_event_loop()
      try:
          # Wait for the result with a maximum timeout of 0.1 seconds
          user_input = await asyncio.wait_for(loop.run_in_executor(None, input, "Enter something: "), timeout=1)
          return bool(user_input)
      except asyncio.TimeoutError:
          # Timeout occurred, handle accordingly
          return False
      except KeyboardInterrupt:
          # Handle KeyboardInterrupt (e.g., when the user presses Ctrl+C)
          return False

   def save(self):
        try:
            user_input = asyncio.run(self.get_input())
            self.check.data = user_input
            self.save_pub.publish(self.check)
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard Interrupt')


def main(args=None):
  rclpy.init(args=args)
  node = GPS_Save()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt')
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
	main()
