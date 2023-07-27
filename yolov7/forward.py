#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from scipy.stats import mode
import rclpy

from std_msgs.msg import Int32MultiArray
from webcam_ros2 import Int32MultiArray


class YoloPub:
    def __init__(self, class_map, queue_size, thresh):
        self.queue_size = queue_size
        self.threshold = thresh
        self.callback_flag = False

        self.queue_list = [[-1 for _ in range(self.queue_size)] for _ in range(len(class_map))]

        self.id_to_queue_list = [self.queue_list[i] for i in range(len(class_map)) for _ in range(len(class_map[i]))]

        self.node = rclpy.create_node('yolo_pub')

        self.forward_pub = self.node.create_publisher(Int32MultiArray, '/forward_sign', 15)
        self.bounding_boxes_sub = self.node.create_subscription(
            Int32MultiArray,
            '/bounding_boxes',
            self.bounding_boxes_callback,
            15
        )

    def hard_vote(self, queue):   # hard vote: 최빈값 찾기
        return int(mode(queue)[0])

    def majority_vote(self, queue):
        candidate = -1
        votes = 0

        for i in range(self.queue_size):
            if votes == 0:
                candidate = queue[i]
                votes = 1
            else:
                votes = votes + 1 if queue[i] == candidate else votes - 1

        count = 0
        for i in range(self.queue_size):
            if queue[i] == candidate:
                count += 1

        return candidate if count > self.queue_size // 2 else -1

    def msg_pub(self):
        final_check = Int32MultiArray()
        queue_list = self.queue_list

        for idx in range(len(queue_list)):
            final_check.data.append(self.hard_vote(queue_list[idx]))

        self.forward_pub.publish(final_check)
        self.callback_flag = False

    def bounding_boxes_callback(self, data):
        queue_size = self.queue_size

        # Process the received bounding boxes data here
        # You can access the data using data.data

        for bounding_box in data.data:
            if bounding_box >= self.threshold:
              if bounding_box < len(self.id_to_queue_list):
                self.id_to_queue_list[bounding_box.id].append(bounding_box.id)

        for queue in self.queue_list:
            if len(queue) == queue_size:
                queue.append(-1)
            while len(queue) != queue_size:
                del queue[0]
        self.callback_flag = True


def main(args=None):
    rclpy.init(args=args)

    CLASS_MAP = (
        ("green", "left", "red", "straightleft", "yellow",),
        ("traffic",)
    )
    QUEUE_SIZE = 13
    ACCURACY_THRESHOLD = 0.7

    node = YoloPub(CLASS_MAP, QUEUE_SIZE, ACCURACY_THRESHOLD)

    while rclpy.ok():
        rclpy.spin_once(node.node)
        if node.callback_flag:
            node.msg_pub()

    node.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
