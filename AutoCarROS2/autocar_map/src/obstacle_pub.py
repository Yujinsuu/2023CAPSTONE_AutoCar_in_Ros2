#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float32, String
from autocar_msgs.msg import Object, ObjectArray
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

from autocar_nav.quaternion import euler_from_quaternion
from autocar_nav.transform_to_matrix import transform_to_matrix

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ObstaclePub(Node):
    def __init__(self):
        super().__init__('obs_pub')

        self.obs_pub = self.create_publisher(ObjectArray, '/obstacles', 10)
        self.delivery_stop_pub = self.create_publisher(Float32, '/delivery_stop', 10)
        self.target_range_pub = self.create_publisher(Marker, '/rviz/target_range', 10)
        self.angle_range_pub = self.create_publisher(Marker, '/rviz/angle_range', 10)

        self.sub_cluster = self.create_subscription(MarkerArray, '/markers', self.cluster_callback, 10)
        self.mode_sub = self.create_subscription(String, '/yolo_mode', self.mode_cb, 10)
        self.angle_sub = self.create_subscription(Float32, '/sign_angle', self.delivery_cb, 10)

        self.GtoL = 1.29 # gps to lidar distance
        self.msg = MarkerArray()
        self.cam_x = -1.35
        self.cam_y = -0.17
        self.angle = 0.0
        self.signs = []
        self.mode = 'None'
        self.visual_angle = []
        self.search_angle = []

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

        try:
            now = rclpy.time.Time()
            t = self.tf_buffer.lookup_transform(world_frame, detection_frame, now)# Duration(seconds=1))
        except:
            self.get_logger().info("can't transform")
            t = TransformStamped()

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
        o.length = abs(xmax - xmin)
        o.width  = abs(ymax - ymin)
        # 장애물 최소크기 설정
        if o.length < 0.4 and o.width < 0.4:
            o.width = 0.4
            o.length = 0.4

        # object x, y, yaw
        x = (xmin + xmax)/2
        y = (ymin + ymax)/2
        yaw = 0.0

        # Compare Slope
        if self.angle != 0.0 and o.length < 0.5:
            slope_min = np.arctan2(y-self.cam_y, xmin-self.cam_x)
            self.search_angle.append(xmin)
            self.search_angle.append(y)
            slope_max = np.arctan2(ymax-self.cam_y, xmin-self.cam_x)
            self.search_angle.append(xmin)
            self.search_angle.append(ymax)
            if slope_min <= self.angle <= slope_max and x <= 10:
                self.signs.append(x)

        # transformation (car -> map)
        o.x, o.y, o.yaw = self.change_frame(x, y, yaw, self.world_frame, self.detection_frame)
        o.header.stamp = self.get_clock().now().to_msg()

        return o


    def msg_pub(self):
        markers = self.msg
        msg = ObjectArray()

        self.visual_angle = []
        self.search_angle = [0.0, 0.0]
        m = np.tan(self.angle)
        if m == 0: m = -1e-2
        # y = m(x-cam_x) + cam_y
        # x = cam_x + (y - cam_y)/m
        y0 = -3
        x0 = self.cam_x + (y0 - self.cam_y)/m

        self.visual_angle.append(x0)
        self.visual_angle.append(y0)

        self.signs = []
        if markers.markers:
            for id, obs in enumerate(markers.markers):
                o = self.get_object(id, obs)
                msg.object_list.append(o)

        self.visual_pub(self.visual_angle, 'target')
        self.visual_pub(self.search_angle, 'list')

        sign = Float32()

        if self.mode != 'delivery' or self.angle == 0.0:
            sign.data = -1.0

        elif len(self.signs):
            sign.data = min(self.signs) / 0.2

        else:
            sign.data = -1.0
        self.delivery_stop_pub.publish(sign)
        self.obs_pub.publish(msg)

    def visual_pub(self, xy, s):
        num = int(len(xy) / 2)

        m = Marker()
        m.header.frame_id = "car"
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD

        point1 = Point()
        point1.x = float(self.cam_x)
        point1.y = float(self.cam_y)
        point1.z = 0.0

        for i in range(num):
            m.points.append(point1)

            point2 = Point()
            point2.x = float(xy[2*i])
            point2.y = float(xy[2*i+1])
            point2.z = 0.0

            m.points.append(point2)

        if s == 'target':
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2

            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0

            self.target_range_pub.publish(m)

        else:
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1

            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = 1.0

            self.angle_range_pub.publish(m)


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
