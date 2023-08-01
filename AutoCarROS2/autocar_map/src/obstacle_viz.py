#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from autocar_msgs.msg import ObjectArray
from visualization_msgs.msg import Marker, MarkerArray

from autocar_nav.quaternion import yaw_to_quaternion


class ObsMarkerPub(Node):
    def __init__(self):
        super().__init__('obs_viz')
        self.obstacle_msg = [] # [(x, y, yaw, L, W), (x, y, yaw, L, W), ... ]

        # self.sub_car = self.create_subscription('/objects/car_1', Object, self.callback_car)
        self.sub_obstacle = self.create_subscription(ObjectArray, 'obstacles', self.callback_obstacle, 1)

        self.obstacle_marker_pub = self.create_publisher(MarkerArray, "/rviz/obstacles", 1)

    def callback_obstacle(self, msg):
        self.obstacle_msg = [(o.x, o.y, o.yaw, o.length, o.width) for o in msg.object_list]
        self.collision_check_and_publish()

    def get_marker_msg(self, obs, id):
        m = Marker()
        m.header.frame_id = "/map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.id = id
        m.type = m.CUBE

        m.pose.position.x = obs[0]
        m.pose.position.y = obs[1]
        m.pose.position.z = 0.75
        quat = yaw_to_quaternion(obs[2])
        m.pose.orientation.x = quat.x
        m.pose.orientation.y = quat.y
        m.pose.orientation.z = quat.z
        m.pose.orientation.w = quat.w

        m.scale.x = obs[3]
        m.scale.y = obs[4]
        m.scale.z = 1.645

		# yellow
        m.color.r = 255 / 255.0
        m.color.g = 204 / 255.0
        m.color.b = 102 / 255.0
        m.color.a = 0.97

        m.lifetime = Duration(nanoseconds=150000000).to_msg()
        return m


    def collision_check_and_publish(self):
        marray = MarkerArray()
        obs_msg = self.obstacle_msg

        if obs_msg:
            for idx, obs in enumerate(obs_msg):
                marker = self.get_marker_msg(obs, idx)
                marray.markers.append(marker)

        self.obstacle_marker_pub.publish(marray)


def main(args=None):
    rclpy.init(args=args)

    try:
        obs_viz = ObsMarkerPub()
        rclpy.spin(obs_viz)
    finally:
        obs_viz.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
