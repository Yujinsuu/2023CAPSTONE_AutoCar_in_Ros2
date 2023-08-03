#!/usr/bin/env python3

import cv2
import argparse
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class Camera_Pub(Node):

    def __init__(self):

        super().__init__('camera_pub')

        image_topic = opt.ns + '/image_raw'
        self.image_pub = self.create_publisher(Image, image_topic, 10)

        source = '/dev/video' + str(opt.source)
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            print('Failed to open Camera')
        else:
            print('Success to open Camera')


    def camera_read(self):
        bridge = CvBridge()

        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Camera_Pub()

    try:
        while True:
            node.camera_read()

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt')

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--ns', type=str, default='/lane')
    parser.add_argument('--source', type=int, default=0)
    opt = parser.parse_args()

    main()
