#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TransformStamped
from autocar_msgs.msg import Object, ObjectArray
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32, String

from autocar_nav.quaternion import euler_from_quaternion
from autocar_nav.transform_to_matrix import transform_to_matrix

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ObstaclePub(Node):
    def __init__(self):
        super().__init__('obs_pub')

        self.obs_pub = self.create_publisher(ObjectArray, '/obstacles', 10)
        self.delivery_stop_pub = self.create_publisher(Float32, '/delivery_stop', 10)

        self.sub_cluster = self.create_subscription(MarkerArray, '/markers', self.cluster_callback, 10)
        self.mode_sub = self.create_subscription(String, '/yolo_mode', self.mode_cb, 10)
        self.angle_sub = self.create_subscription(Float32, '/sign_angle', self.delivery_cb, 10)

        self.msg = MarkerArray()
        self.cam_x = -1.39
        self.cam_y = -0.20
        self.angle = 0.0
        self.signs = []
        self.mode = 'None'

        self.world_frame = "odom"
        self.detection_frame = "car"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.msg_pub)

    def change_frame(self, x, y, yaw, world_frame, detection_frame):
        pose = np.array([[1.0, 0.0, 0.0, x],
                         [0.0, 1.0, 0.0, y],
                         [0.0, 0.0, 1.0, 0.0],
                         [0.0, 0.0, 0.0, 1.0]])
        now = rclpy.time.Time()
        a = self.tf_buffer.can_transform(world_frame, detection_frame, now, Duration(seconds=1))
        if a:
            t = self.tf_buffer.lookup_transform(world_frame, detection_frame, rclpy.time.Time(), Duration(seconds=1))
        else:
            t = TransformStamped()


        # a = self.tf_buffer.can_transform(world_frame, detection_frame, now, Duration(seconds=1))
        # while(True):
        #     if a:
        #         t = self.tf_buffer.lookup_transform(world_frame, detection_frame, now)
        #         break
        #     else:
        #         now = rclpy.time.Time()
        #         a = self.tf_buffer.can_transform(world_frame, detection_frame, now, Duration(seconds=1))
        #         self.get_logger().info('stuck!')


        # try:
        #     t = self.tf_buffer.lookup_transform(world_frame, detection_frame, rclpy.time.Time(), Duration(seconds=1)) #
        # except (LookupException, ConnectivityException, ExtrapolationException):
        #     pass
        # self.get_logger().info('%f' % t.transform.rotation.z)


        tf_matrix = transform_to_matrix(t)
        yaw = euler_from_quaternion(t.transform.rotation.x,
                                    t.transform.rotation.y,
                                    t.transform.rotation.z,
                                    t.transform.rotation.w)
        result = np.array(np.dot(tf_matrix, pose))
        # euler = tf.transformations.euler_from_matrix(result)

        return float(result[0, 3]), float(result[1, 3]), yaw  #, euler[2]


    def cluster_callback(self, data):
        self.msg = data

    def mode_cb(self, msg):
        self.mode = msg.data

    def delivery_cb(self, msg):
        self.angle = msg.data

    def get_object(self, id, obs):
        o = Object()

        o.header.frame_id = "map"
        o.id = id
        o.classification = o.CLASSIFICATION_CAR

        xmin = obs.points[1].x
        xmax = obs.points[0].x
        ymin = obs.points[3].y
        ymax = obs.points[4].y

        # object length, width
        o.length = abs(obs.points[0].x - obs.points[1].x)
        o.width = abs(obs.points[4].y - obs.points[3].y)

        # object x, y, yaw
        x = (obs.points[0].x + obs.points[1].x)/2
        y = (obs.points[4].y + obs.points[3].y)/2
        yaw = 0.0

        # Compare Slope
        xmin = 0 if xmin < 0 else xmin
        slope_min = np.arctan2(ymin-self.cam_y, xmin-self.cam_x)
        slope_max = np.arctan2(ymax-self.cam_y, xmin-self.cam_x)
        if slope_min <= self.angle <= slope_max and x <= 10:
            self.signs.append(x + 0.2)

        # transformation (car -> map)
        o.x, o.y, o.yaw = self.change_frame(x, y, yaw, self.world_frame, self.detection_frame)
        o.header.stamp = self.get_clock().now().to_msg()

        return o


    def msg_pub(self):
        markers = self.msg
        msg = ObjectArray()

        self.signs = []
        if markers.markers:
            for id, obs in enumerate(markers.markers):
                o = self.get_object(id, obs)
                msg.object_list.append(o)

        sign = Float32()
        if len(self.signs):
            sign.data = min(self.signs) / 0.2

        if self.mode != 'delivery' or self.angle == 0.0:
            sign.data = -1.0

        self.delivery_stop_pub.publish(sign)
        self.obs_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        obs_pub = ObstaclePub()
        rclpy.spin(obs_pub)
    finally:
        obs_pub.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
	main()
