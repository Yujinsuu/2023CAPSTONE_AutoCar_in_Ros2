#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from ultrafastLaneDetector import UltrafastLaneDetector, ModelType

class LaneNet(Node):
    def __init__(self):
        super().__init__('lanenet')

        self.steer_pub = self.create_publisher(Float32, "/lanenet_steer", 10)
        self.image_pub = self.create_publisher(Image, "/lanenet_image", 10)

        # self.mode_sub = self.create_subscription(String, "/yolo_mode", self.mode_cb, 10)
        self.image_sub = self.create_subscription(Image, "/image_raw", self.image_cb, 10)

        self.mode = 'None'
        self.steer_angle = Float32()

        model_path = 'models/culane_18.pth'
        model_type = ModelType.CULANE
        use_gpu = True

        # Initialize lane detection model
        self.lane_detector = UltrafastLaneDetector(model_path, model_type, use_gpu)


    def mode_cb(self, msg):
        self.mode = msg.data


    def image_cb(self, img):
        bridge = CvBridge()
        cap = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

        # if self.mode == 'tunnel':
        #     result = self.lane_detector.detect_lanes(cap)
        #     image_message = bridge.cv2_to_imgmsg(result, encoding="bgr8")

        #     self.steer_angle.data = ?
        # else:
        #     image_message = bridge.cv2_to_imgmsg(cap, encoding="bgr8")

        frame = self.lane_detector.detect_lanes(cap)
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        self.steer_pub.publish(self.steer_angle)

        image_message.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(image_message)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LaneNet()

        rclpy.spin(node)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
