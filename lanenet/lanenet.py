#!/usr/bin/env python3

import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image

from ultrafastLaneDetector import UltrafastLaneDetector, ModelType


class LaneNet(Node):

    def __init__(self):

        super().__init__('lanenet')

        self.steer_pub = self.create_publisher(Float64MultiArray, "/lanenet_steer", 10)
        self.image_pub = self.create_publisher(Image, "/lanenet_image", 10)

        # self.mode_sub = self.create_subscription(String, "/yolo_mode", self.mode_cb, 10)
        self.image_sub = self.create_subscription(Image, "/lane/image_raw", self.image_cb, 10)

        self.mode = 'None'
        self.steer_angle = Float64MultiArray()
        self.K = 0.1

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

        width, height = cap.shape[1], cap.shape[0]

        padding_size = 175
	    # 좌측에 흰색 이미지 추가
        padding_image = np.zeros((height, padding_size, 3), dtype=np.uint8) * 255  # 흰색 이미지 생성
        # frame 좌우로 확장
        cap = np.concatenate((padding_image, cap), axis=1)
        cap = np.concatenate((cap, padding_image), axis=1)

        check, result, steer_angle = self.lane_detector.detect_lanes(cap)
        image_message = bridge.cv2_to_imgmsg(result, encoding="bgr8")
        image_message.header.stamp = self.get_clock().now().to_msg()

        self.image_pub.publish(image_message)

        steer_angle = np.deg2rad(-steer_angle)
        self.steer_angle.data = [check, self.K * steer_angle]

        self.steer_pub.publish(self.steer_angle)


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
