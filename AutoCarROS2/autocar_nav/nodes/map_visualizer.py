#!/usr/bin/env python3

import os

import numpy as np
import pandas as pd
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from autocar_msgs.msg import Path2D

class Converter(Node):

    def __init__(self, file_=None, r=255/255.0, g=255/255.0, b=255/255.0, a= 0.5, scale=0.1):
        super().__init__('global_map')

        # Initialise suscriber(s)
        self.localisation_sub = self.create_subscription(Path2D, '/autocar/path', self.path_pub, 10)

        # Initialise publisher(s)
        self.link_pub = self.create_publisher(MarkerArray, '/rviz/lane_links', 10)

        self.r = r
        self.g = g
        self.b = b
        self.a = a
        self.scale = scale
        self.ma = None
        self.points = 0

        if file_ is not None:
            self.file = file_
            df = pd.read_csv(file_)
            ax = df['X-axis'].values.tolist()
            ay = df['Y-axis'].values.tolist()
            points = min(len(ax), len(ay))
            self.points = points

            if points != 0:
                self.make_marker_array(ax,ay)

    def make_marker_array(self, ax, ay):
        ma = MarkerArray()

        for i in range(self.points):
            m = Marker()
            m.id = i
            m.header.frame_id = "map"
            m.type = m.SPHERE
            m.action = m.ADD

            m.scale.x = self.scale
            m.scale.y = self.scale
            m.scale.z = self.scale

            m.color.r = self.r
            m.color.g = self.g
            m.color.b = self.b
            m.color.a = self.a

            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0

            m.pose.position.x = ax[i]
            m.pose.position.y = ay[i]
            m.pose.position.z = 0.0

            ma.markers.append(m)

        self.ma = ma

    def path_pub(self, msg):
        self.link_pub.publish(self.ma)

def main(args=None):

    # Initialise the node
    rclpy.init(args=args)

    # Get path to waypoints.csv
    link_file = os.path.join(get_package_share_directory('autocar_nav'), 'data', 'output.csv')

    try:
        # Initialise the class
        link_cv = Converter(link_file, r=250/255.0, g=236/255.0, b=10/255.0, a=0.5, scale=0.5)

        # Stop the node from exiting
        rclpy.spin(link_cv)

    finally:
        link_cv.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
