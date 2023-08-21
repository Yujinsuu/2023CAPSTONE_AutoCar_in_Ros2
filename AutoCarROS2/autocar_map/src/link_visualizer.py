#!/usr/bin/env python3

import os
import sys
import random

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

path_module = os.path.join(get_package_share_directory('autocar_map'), 'path')
sys.path.append(path_module)
from path_link import *

class Converter(Node):

    def __init__(self):
        super().__init__('mapviz')

        self.base_map = use_map.base_map
        self.global_map_x = use_map.global_map_x
        self.global_map_y = use_map.global_map_y
        self.map_x = use_map.ax
        self.map_y = use_map.ay

        self.count = {'global':0, 'parking':0, 'revpark':0}
        for keys in self.map_x.keys():
            key = keys.split('_')[0]

            if key not in self.count:
                self.count[key] = 0

            self.count[key] += 1

        self.red = []
        self.green = []
        self.blue = []
        for _ in range(self.count['global']):
            r = random.random()
            g = random.random()
            b = random.random()
            self.red.append((135*r+120)/255)
            self.green.append((135*g+120)/255)
            self.blue.append((135*b+120)/255)

        self.base_pub = self.create_publisher(MarkerArray, '/rviz/base_links', 10)
        self.link_pub = self.create_publisher(MarkerArray, '/rviz/global_links', 10)

        self.timer = self.create_timer(1, self.make_path)

    def make_path(self):
        self.map_b = MarkerArray()
        self.map = MarkerArray()
        self.seq_ = 0

        for i in range(len(self.base_map)):
            bx = self.base_map[i]['X-axis']
            by = self.base_map[i]['Y-axis']
            self.make_marker_array('B',len(bx), bx, by, r=1.0, g=1.0, b=1.0, a=0.5, scale=0.3)


        if self.count['global'] != 0:
            for i in range(self.count['global']):
                mx = self.global_map_x['global_' + str(i)]
                my = self.global_map_y['global_' + str(i)]
                self.make_marker_array('G', len(mx), mx, my, r=self.red[i], g=self.green[i], b=self.blue[i], a=0.5, scale=0.5)

        if self.count['parking'] != 0:
            for i in range(self.count['parking']):
                mx = self.map_x['parking_'+str(i)]
                my = self.map_y['parking_'+str(i)]
                self.make_marker_array('P', len(mx), mx, my, r=0.08, g=0.98, b=0.72, a=0.5, scale=0.5)

        if self.count['revpark'] != 0:
            for i in range(self.count['revpark']):
                mx = self.map_x['revpark_'+str(i)]
                my = self.map_y['revpark_'+str(i)]
                self.make_marker_array('R', len(mx), mx, my, r=0.8, g=0.08, b=0.98, a=0.5, scale=0.5)

        self.link_pub.publish(self.map)
        self.base_pub.publish(self.map_b)

    def make_marker_array(self, mode, num, ax, ay, r, g, b, a, scale):
        if mode == 'B':
            points = []
            for i in range(num):
                points.append(Point(x=ax[i], y=ay[i], z=0.0))

        for i in range(num):
            m = Marker()
            m.header.frame_id = "map"
            m.id = self.seq_
            if mode == 'B': m.type = m.LINE_STRIP
            else: m.type = m.SPHERE
            m.action = m.ADD

            m.scale.x = scale
            if mode != 'B':
                m.scale.y = scale
                m.scale.z = scale

                m.pose.orientation.x = 0.0
                m.pose.orientation.y = 0.0
                m.pose.orientation.z = 0.0
                m.pose.orientation.w = 1.0

                m.pose.position.x = ax[i]
                m.pose.position.y = ay[i]
                m.pose.position.z = 0.0

            else:
                m.points = points

            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a


            self.seq_ += 1

            if mode == 'B': self.map_b.markers.append(m)
            else: self.map.markers.append(m)


def main(args=None):

    # Initialise the node
    rclpy.init(args=args)

    try:
        # Initialise the class
        link_cv = Converter()

        # Stop the node from exiting
        rclpy.spin(link_cv)

    finally:
        link_cv.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
