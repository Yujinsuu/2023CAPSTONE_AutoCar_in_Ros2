import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher_node')
        self.publisher = self.create_publisher(Image, '/side/image_raw', 10)
        self.timer = self.create_timer(0.05, self.publish_video)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('data/delivery_A.mp4')

    def publish_video(self):
        if(self.cap.get(cv2.CAP_PROP_POS_FRAMES) == self.cap.get(cv2.CAP_PROP_FRAME_COUNT)):
            self.cap.open('data/delivery_A.mp4')
        ret, frame = self.cap.read()
        if not ret:
            self.cap.release()
            rclpy.shutdown()
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
