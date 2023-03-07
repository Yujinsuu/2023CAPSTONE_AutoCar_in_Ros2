#!/usr/bin/env python3

import os

import numpy as np
import pandas as pd
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from autocar_msgs.msg import Path2D

file_path = os.path.join(get_package_share_directory('autocar_map'), 'data')

class MakingPath:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []

class Path(Node):
    def __init__(self, file_=None):

        self.ax = {'global':{},'mission':{}}
        self.ay = {'global':{},'mission':{}}
        self.link_wp = {'global':{},'mission':{}}
        self.wp_num = {'global':{},'mission':{}}

        self.file = file_

        if file_ is not None:
            df = pd.read_csv(file_)
            ax = df['X-axis'].values.tolist()
            self.ax['global'] = ax
            ay = df['Y-axis'].values.tolist()
            self.ay['global'] = ay
            points = min(len(ax), len(ay))
            self.wp_num['global'] = points

        self.target_speed = []
        self.link_change = []
        self.car_mode = []
        self.mission_map_num = 0

    # link_wp 안에 마지막 waypoint 추가 및 0 제거하는 함수
    def set_global_link(self, wp_list=[0], mode='global'):
        wp_list.append(self.wp_num[mode])
        self.link_wp[mode] = wp_list

def linear_track():
    link_file = file_path + '/straight.csv'
    linear_track = Path(link_file)
    linear_track.link_change = [405]
    linear_track.car_mode = ['G','E']
    linear_track.set_global_link(linear_track.link_change)

    return linear_track

use_map = linear_track()
start_index = 0
