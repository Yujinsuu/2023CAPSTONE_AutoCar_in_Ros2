#!/usr/bin/env python3

import os
import sys
import numpy as np
from collections import deque, Counter
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

        super().__init__('parking_map')

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
                self.parking_x[key1] = use_map.ax[keys][:-6]
                self.parking_y[key1] = use_map.ay[keys][:-6]

            elif key0 == 'revpark':
                self.revpark_x[key1] = use_map.ax[keys][:-8]
                self.revpark_y[key1] = use_map.ay[keys][:-8]

        self.parking_num = len(self.parking_x) - 1
        self.revpark_num = len(self.revpark_x) - 1
        self.P = [14, 3] # kcity
        # self.P = [5, 4] # htech
        # self.P = [9, 4] # boong
        self.R = [20, 5] # kcity
        # self.R = [7, 7] # htech
        # self.R = [22, 7] # boong

        self.path_check = False
        self.path = -1
        self.prev_path = -1

        queue_size = 9
        init_queue = [0 for _ in range(queue_size)]
        self.queue = deque(init_queue, maxlen = queue_size)
        self.rev_check = []

        self.mode = 'global'

        self.car_width = 1.2
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
            if not self.path_check and wp >= self.P[0] + self.P[1]*0:
                path = max(min(int((wp - self.P[0]) / self.P[1]), self.parking_num), 0)
                if path != self.prev_path:
                    queue_size = 9
                    init_queue = [0 for _ in range(queue_size)]
                    self.queue = deque(init_queue, maxlen = queue_size)

                if not self.parking_collision_check(self.parking_x[str(path)], self.parking_y[str(path)]):
                    self.queue.append(1)
                else:
                    self.queue.append(0)

                counter = Counter(self.queue)
                value, count = counter.most_common(1)[0]

                if value == 1:
                    self.path_check = True
                    self.path = path

                self.prev_path = path

        elif self.mode == 'revpark':
            if not self.path_check and wp >= self.R[0] + self.R[1]*0:
                path = max(min(int((wp - self.R[0]) / self.R[1]), self.revpark_num), 0)
                if path != self.prev_path:
                    queue_size = 13
                    init_queue = [0 for _ in range(queue_size)]
                    self.queue = deque(init_queue, maxlen = queue_size)
                    self.rev_check = []

                if not self.parking_collision_check(self.revpark_x[str(path)], self.revpark_y[str(path)]):
                    self.queue.append(1)
                else:
                    self.queue.append(0)
                    self.rev_check.append(False)

                if len(self.rev_check) > 20:
                    queue_size = 13
                    init_queue = [0 for _ in range(queue_size)]
                    self.queue = deque(init_queue, maxlen = queue_size)

                counter = Counter(self.queue)
                value, count = counter.most_common(1)[0]

                if value == 1:
                    self.path_check = True
                    self.path = path

                self.prev_path = path

        else:
            self.path_check = False
            self.path = -1
            self.prev_path = -1

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

        if self.mode == 'parking' and self.path > self.parking_num:
            path_idx.data = -1

        elif self.mode == 'revpark' and self.path > self.revpark_num:
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
