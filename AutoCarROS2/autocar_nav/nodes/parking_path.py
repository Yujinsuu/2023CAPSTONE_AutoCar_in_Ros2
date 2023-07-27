#!/usr/bin/env python3

import os
import sys
import numpy as np
from scipy.interpolate import CubicSpline

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Int32
from autocar_msgs.msg import LinkArray, State2D, ObjectArray

from autocar_nav.separation_axis_theorem import separating_axis_theorem, get_vertice_rect

path_module = os.path.join(get_package_share_directory('autocar_map'), 'path')
sys.path.append(path_module)
from path_link import *


class ParkingPath(Node):

    def __init__(self):

        ''' Class constructor to initialise the class '''

        super().__init__('parking_path')

        # Initialise publisher(s)
        self.park_path_pub = self.create_publisher(Int32, '/autocar/parking_path', 10)

        # Initialise suscriber(s)
        self.localisation_sub = self.create_subscription(State2D, '/autocar/state2D', self.vehicle_state_cb, 10)
        self.links_sub = self.create_subscription(LinkArray, '/autocar/mode', self.link_cb, 10)
        self.obstacle_sub = self.create_subscription(ObjectArray, '/obstacles', self.obstacle_cb, 10)

        self.parking_x = {}
        self.parking_y = {}
        self.revpark_x = {}
        self.revpark_y = {}

        for keys in use_map.ax.keys():
            key0 = keys.split('_')[0]
            key1 = keys.split('_')[1]

            if key0 == 'parking':
                self.parking_x[key1] = use_map.ax[keys]
                self.parking_y[key1] = use_map.ay[keys]

            elif key0 == 'revpark':
                self.revpark_x[key1] = use_map.ax[keys]
                self.revpark_y[key1] = use_map.ay[keys]

        self.parking_num = len(self.parking_x) - 1
        self.revpark_num = len(self.revpark_x) - 1

        self.path_check = False
        self.path = -1

        self.mode = 'global'

        self.car_width = 1.2
        self.obs_list = {'0':[-238.0, 518.0, 3.0],
                         '1':[-235.0, 516.0, 3.0],
                         '2':[-231.0, 531.0, 3.0],
                         '3':[32.0, 85.0, 3.0],
                         '4':[39.0, 88.3, 3.0],
                         '5':[35.0, 90.0, 3.0],
                         '6':[37.0, 93.0, 3.0],
                         '7':[-99.0, 620.0, 3.0],
                         '8':[-102.0, 623.0, 3.0],
                         '9':[-105.0, 623.0, 3.0],
                         }
        self.obstacles = []

        self.timer = self.create_timer(0.1, self.park_path_publish)


    def vehicle_state_cb(self, msg):
        '''
        Callback function to recieve vehicle state information from localization in global frame
        '''
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta


    def link_cb(self, msg):
        wp = msg.closest_wp
        self.mode = msg.mode

        if self.mode == 'parking':
            if not self.path_check and wp >= 7 + 4*0:
                path = max(min(int((wp - 7) / 4), self.parking_num), 0)
                if not self.parking_collision_check(self.parking_x[str(path)], self.parking_y[str(path)]):
                    self.path_check = True
                    self.path = path

        elif self.mode == 'revpark':
            if not self.path_check and wp >= 15 + 9*0:
                path = max(min(int((wp - 15) / 9), self.revpark_num), 0)
                if not self.parking_collision_check(self.revpark_x[str(path)], self.revpark_y[str(path)]):
                    self.path_check = True
                    self.path = path

        else:
            self.path_check = False
            self.path = -1

    def obstacle_cb(self, msg):
        self.obstacles = [(o.x, o.y, o.yaw, o.length, o.width) for o in msg.object_list]

    def parking_collision_check(self, ax, ay):
        cx_ = CubicSpline(range(len(ax)), ax)
        cy_ = CubicSpline(range(len(ay)), ay)
        dx = cx_(np.arange(0, len(ax) - 1, 0.5))
        dy = cy_(np.arange(0, len(ay) - 1, 0.5))
        cyaw = np.arctan2(dy[1:] - dy[:-1], dx[1:] - dx[:-1])
        cx = dx[:-1]
        cy = dy[:-1]

        car_msg = []
        for i in range(0,len(cyaw)):
            car_msg.append((cx[i], cy[i], cyaw[i], 1.5, 1.5)) # x, y, yaw, length, width
            for obs in self.obstacles:
                for car in car_msg:
                    car_vertices = get_vertice_rect(car)
                    obstacle_vertices = get_vertice_rect(obs)
                    is_collide = separating_axis_theorem(car_vertices, obstacle_vertices)

                    if is_collide:
                        return True
        return False

    def park_path_publish(self):

        path_idx = Int32()
        # path_idx.data = self.path_num

        if self.mode == 'parking' and self.path >= self.parking_num:
            path_idx.data = -1

        elif self.mode == 'revpark' and self.path >= self.revpark_num:
            path_idx.data = -1

        else:
            path_idx.data = self.path


        self.park_path_pub.publish(path_idx)

def main(args=None):

    # Initialise the node
    rclpy.init(args=args)

    try:
        # Initialise the class
        park = ParkingPath()

        # Stop the node from exiting
        rclpy.spin(park)

    finally:
        park.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
