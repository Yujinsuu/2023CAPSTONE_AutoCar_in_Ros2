#!/usr/bin/env python3

import os
import sys
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Float64MultiArray, Int32, String
from autocar_msgs.msg import Path2D, State2D, LinkArray, Obstacle
from geometry_msgs.msg import Pose2D

from autocar_nav.calculate_offset import point_offset, line_offset
from autocar_nav.calculate_curvature import classify_segments

path_module = os.path.join(get_package_share_directory('autocar_map'), 'path')
sys.path.append(path_module)
from path_link import *

class GlobalPathPlanner(Node):

    def __init__(self):

        ''' Class constructor to initialise the class '''

        super().__init__('global_planner')

        # Initialise publisher(s)
        self.goals_pub = self.create_publisher(Path2D, '/autocar/goals', 10)
        self.lanes_pub = self.create_publisher(Path2D, '/autocar/tunnel_lane', 10)
        self.links_pub = self.create_publisher(LinkArray, '/autocar/mode', 10)
        self.mode_pub = self.create_publisher(String, '/yolo_mode', 10)
        # self.offset_pub = self.create_publisher(Float64MultiArray, '/autocar/tunnel_offset', 10)

        # Initialise suscriber(s)
        self.localization_sub = self.create_subscription(State2D, '/autocar/state2D', self.vehicle_state_cb, 10, callback_group=ReentrantCallbackGroup())
        self.parking_path_sub = self.create_subscription(Int32, '/autocar/parking_path', self.parking_path_cb, 10)
        self.mission_status_sub = self.create_subscription(String,'/autocar/mission_status', self.mission_status_cb, 10)
        self.walls_sub = self.create_subscription(Path2D, '/wall_path', self.walls_cb, 10)
        self.lanes_sub = self.create_subscription(Path2D, '/wall_lane', self.lanes_cb, 10)
        self.obstacle_sub = self.create_subscription(Obstacle, '/autocar/obs_recog', self.obstacle_cb, 10)

        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', None),
                    ('waypoints_ahead', None),
                    ('waypoints_behind', None),
                    ('passed_threshold', None),
                    ('waypoints', None),
                    ('centreofgravity_to_frontaxle', None)
                ]
            )

            self.passed_threshold = float(self.get_parameter("passed_threshold").value)
            self.FL = float(self.get_parameter("centreofgravity_to_frontaxle").value)
            self.frequency = float(self.get_parameter("update_frequency").value)
            self.ds = 1/self.frequency

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class variables to use whenever within the class when necessary
        self.x = None
        self.y = None
        self.theta = None

        # Import waypoints.csv into class variables ax and ay
        self.mx = use_map.ax
        self.my = use_map.ay

        self.global_index = start_index
        self.link = 'global_' + str(self.global_index)
        self.mode_list = use_map.car_mode
        self.mode = 'global'

        self.status = 'driving'
        self.direction = 'Straight'
        self.curv_thresh = 0.01

        self.next_path = use_map.next_path

        self.count = {}
        for keys in self.mx.keys():
            key = keys.split('_')[0]

            if key not in self.count:
                self.count[key] = 0

            self.count[key] += 1

        # Class constants
        self.wp_ahead = 10
        self.wp_behind = 5
        self.wp_published = self.wp_ahead + self.wp_behind
        self.wp_num = len(self.mx[self.link])
        self.closest_wp = 0

        self.traffic_stop_wp = 1e3
        self.parking_stop_wp = 1e3

        self.parking_path_num = -1

        self.tx = []
        self.ty = []
        self.lx = []
        self.ly = []
        self.tunnel_x = []
        self.tunnel_y = []
        self.lane_x = []
        self.lane_y = []
        self.map_fix = False

        self.offset_x = 0.0
        self.offset_y = 0.0
        
        self.obstacle_detected = 0
        self.obstacle = 'None'
        self.obs_distance = float(1e3)


    def parking_path_cb(self, msg):
        self.parking_path_num = msg.data
        if self.mode not in ['parking','revpark']: self.parking_path_num = -1
        if self.status == 'complete': self.parking_path_num = -1


    def mission_status_cb(self, msg):
        self.status = msg.data

    def walls_cb(self, msg):
        self.tx = []
        self.ty = []
        for i in range(len(msg.poses)):
            wx = msg.poses[i].x
            wy = msg.poses[i].y
            self.tx.append(wx)
            self.ty.append(wy)

    def lanes_cb(self, msg):
        self.lx = []
        self.ly = []
        for i in range(len(msg.poses)):
            wx = msg.poses[i].x
            wy = msg.poses[i].y
            self.lx.append(wx)
            self.ly.append(wy)

    def vehicle_state_cb(self, msg):
        '''
            Callback function to update vehicle state

            Parameters:
                self.x          - Represents the current x-coordinate of the vehicle
                self.y          - Represents the current y-coordinate of the vehicle
                self.theta      - Represents the current yaw of the vehicle
        '''
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.theta = msg.pose.theta

        self.set_waypoints()
        
    def obstacle_cb(self, msg):
        self.obstacle_detected = msg.detected
        self.obstacle = msg.obstacle
        self.obs_distance = msg.distance


    def get_closest_waypoints(self):
        # Identify position of vehicle front axle
        fx = self.x + self.FL * np.cos(self.theta)
        fy = self.y + self.FL * np.sin(self.theta)

        self.link = 'global_' + str(self.global_index)

        via_x = self.mx[self.link]
        via_y = self.my[self.link]

        wp_num = len(via_x)
        wp_ahead = 10
        wp_behind = 5

        dx = [fx - icx for icx in via_x] # Find the x-axis of the front axle relative to the path_num
        dy = [fy - icy for icy in via_y] # Find the y-axis of the front axle relative to the path_num

        d = np.hypot(dx, dy)        # Find the distance from the front axle to the path_num
        closest_id = int(np.argmin(d))   # Returns the index with the shortest distance in the array

        transform = self.frame_transform(via_x[closest_id], via_y[closest_id], fx, fy, self.theta)

        if self.mode == 'revpark' and self.parking_path_num != -1:
            parking_stop = [45, 50, 56]
            wp_num = parking_stop[self.parking_path_num] # kcity
            # wp_num = 47 + 5 * self.parking_path_num
        self.traffic_stop_wp = wp_num - wp_ahead - closest_id


        if closest_id >= len(via_x) - wp_ahead:
            if self.global_index < self.count['global'] - 1:
                self.global_index += 1
                closest_id = wp_behind
                self.traffic_stop_wp = 1e3


        self.mode = self.mode_list[self.global_index]
        self.closest_wp = closest_id

        return closest_id, transform


    def get_parking_stop_wp(self, px, py):
        # Identify position of vehicle front axle
        fx = self.x + self.FL * np.cos(self.theta)
        fy = self.y + self.FL * np.sin(self.theta)

        dx = [fx - icx for icx in px]
        dy = [fy - icy for icy in py]

        d = np.hypot(dx, dy)
        closest_id = np.argmin(d)

        return len(px) - closest_id


    def set_waypoints(self):

        closest_id, transform = self.get_closest_waypoints()

        if self.mode in ['tunnel', 'static0', 'static1']:
            self.wp_ahead = 20
            self.wp_behind = 10
        else:
            self.wp_ahead = 10
            self.wp_behind = 5


        if self.mode == 'parking' and self.parking_path_num != -1:
            # 주차 맵 첫 부분과 현재 글로벌 맵의 첫 부분을 잇는 path를 다시 형성
            self.get_logger().info('Parking path number : {} '.format(self.parking_path_num))

            self.link = self.mode + '_' + str(self.parking_path_num)

            point_x0 = self.mx['global_' + str(self.global_index)][0]
            point_x1 = self.mx[self.link][0]
            point_y0 = self.my['global_' + str(self.global_index)][0]
            point_y1 = self.my[self.link][0]

            point0 = [point_x0, point_y0]
            point1 = [point_x1, point_y1]

            px, py = self.interpolate(point0, point1)

            for i in range(len(self.mx[self.link])):
                px.append(self.mx[self.link][i])
                py.append(self.my[self.link][i])

            if len(px) > 20:
                px = px[-20:]
                py = py[-20:]

            self.parking_stop_wp = self.get_parking_stop_wp(px, py)
            self.direction = 'Straight'

        elif self.mode == 'revpark' and self.parking_path_num != -1 and self.status != 'driving':
            self.get_logger().info('Parking path number : {} '.format(self.parking_path_num))

            self.link = self.mode + '_' + str(self.parking_path_num)
            park_x = self.mx[self.link]
            park_y = self.my[self.link]

            d = 0.2 # 주차 경로 d m 앞으로 이동
            # kcity
            park_x = [i - 0.4423572639935003 * d for i in park_x]
            park_y = [i - 0.8308233860879995 * d for i in park_y]
            # htech
            # park_x = [i - 0.256388029315985 * d for i in park_x]
            # park_y = [i - 0.904206226115093 * d for i in park_y]

            point_x0 = self.mx['global_' + str(self.global_index)][-1]
            point_x1 = park_x[0]
            point_y0 = self.my['global_' + str(self.global_index)][-1]
            point_y1 = park_y[0]

            point0 = [point_x0, point_y0]
            point1 = [point_x1, point_y1]

            px, py = self.interpolate(point0, point1)

            for i in range(len(park_x)):
                px.append(park_x[i])
                py.append(park_y[i])

            self.parking_stop_wp = self.get_parking_stop_wp(px, py)
            self.direction = 'Straight'

            px = px[::-1]
            py = py[::-1]

            if len(px) > 30:
                px = px[:30]
                py = py[:30]


        else: # Global and else

            self.link = 'global_' + str(self.global_index)
            self.wp_num = len(self.mx[self.link])
            percent = int( 100 * closest_id / self.wp_num)

            self.get_logger().info('Link : {} ( {} % )'.format(self.link, percent))

            if closest_id < self.wp_behind:
                # If the vehicle is starting along the path_num
                px = self.mx[self.link][0: self.wp_published]
                py = self.my[self.link][0: self.wp_published]

            elif closest_id > (self.wp_num - self.wp_published):
                # If the vehicle is finishing the given set of waypoints
                px = self.mx[self.link][-self.wp_published:]
                py = self.my[self.link][-self.wp_published:]

            elif transform[1] < (0.0 - self.passed_threshold):
                # If the vehicle has passed, closest point is preserved as a point behind the car
                px = self.mx[self.link][closest_id - (self.wp_behind - 1) : closest_id + (self.wp_ahead + 1)]
                py = self.my[self.link][closest_id - (self.wp_behind - 1) : closest_id + (self.wp_ahead + 1)]

            else:
                # If the vehicle has yet to pass, a point behind the closest is preserved as a point behind the car
                px = self.mx[self.link][(closest_id - self.wp_behind) : (closest_id + self.wp_ahead)]
                py = self.my[self.link][(closest_id - self.wp_behind) : (closest_id + self.wp_ahead)]

            self.direction = classify_segments(px[self.wp_behind:], py[self.wp_behind:], self.curv_thresh)

            self.parking_stop_wp = 1e3

            if self.mode == 'tunnel':
                lx,ly,lyaw = [1e6],[1e6],0.0
                if self.status == 'avoid' and self.obs_distance <= 6:
                    self.map_fix = True
                    
                elif self.map_fix == True and self.status == 'lanenet':
                    self.map_fix = False
                
                if (self.status == 'lanenet' or self.map_fix == False) and len(self.tx) != 0 :
                    self.tunnel_x = self.tx
                    self.tunnel_y = self.ty
                    self.lane_x = self.lx
                    self.lane_y = self.ly

                elif self.status == 'avoid' and self.map_fix == True:
                    lx = self.lane_x
                    ly = self.lane_y
                    lyaw = np.arctan2((py[-1]-py[0]),(px[-1]-px[0]))

                if self.status not in ['driving', 'complete']:
                    if len(self.tunnel_x) != 0:
                        px = self.tunnel_x
                        py = self.tunnel_y

                self.publish_lanes(lx,ly,lyaw)

        self.publish_goals(px, py)

    def frame_transform(self, point_x, point_y, axle_x, axle_y, theta):
        '''
            Recieves position of vehicle front axle, and id of closest waypoint. This waypoint is transformed from
            "odom" frame to the vehicle frame

            Arguments:
                closest_id          - Index to closest waypoint to front axle in master waypoint list
                point_x, point_y    - Coordinates (x,y) of target point in world frame
                axle_x, axle_y      - Coordinates (x,y) of vehicle front axle position in world frame
        '''

        c = np.cos(theta)  # Creates rotation matrix given theta
        s = np.sin(theta)  # Creates rotation matrix given theta
        R = np.array(((c, s), (-s, c)))

        p = np.array(((point_x), (point_y)))      # Position vector of closest waypoint (world frame)
        v = np.array(((axle_x), (axle_y)))        # Position vector of vehicle (world frame)
        vp = p - v                                # Linear translation between vehicle and point
        transform = R.dot(vp)                     # Product of rotation matrix and translation vector

        return transform

    def interpolate(self, p0, p1):
        # p0과 p1 사이의 거리 계산
        dist = math.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2)

        # p0과 p1 사이를 1m 간격으로 보간하여 리스트로 저장
        num_points = int(dist)  # 소숫점 이하 버림
        x_values = [p0[0] + i*(p1[0]-p0[0])/dist for i in range(num_points)]
        y_values = [p0[1] + i*(p1[1]-p0[1])/dist for i in range(num_points)]

        return x_values, y_values

    def publish_lanes(self, lx, ly, lyaw):

        waypoints = min(len(lx), len(ly))

        lanes = Path2D()

        for i in range(waypoints):
            # Appending to Target Goals
            lane = Pose2D()
            lane.x = lx[i]
            lane.y = ly[i]
            lane.theta = lyaw

            lanes.poses.append(lane)

        if waypoints != 0: self.lanes_pub.publish(lanes)

    def publish_goals(self, px, py):

        waypoints = min(len(px), len(py))

        goals = Path2D()

        for i in range(waypoints):
            # Appending to Target Goals
            goal = Pose2D()
            goal.x = px[i]
            goal.y = py[i]

            goals.poses.append(goal)

        if waypoints != 0: self.goals_pub.publish(goals)

        links = LinkArray()

        links.link_num = self.global_index
        links.closest_wp = self.closest_wp
        links.mode = self.mode
        links.traffic_stop_wp = int(self.traffic_stop_wp)
        links.parking_stop_wp = int(self.parking_stop_wp)
        links.direction = self.direction
        links.next_path = self.next_path[self.global_index]

        self.links_pub.publish(links)

        to_yolo = String()
        if self.mode == 'tunnel':
            to_yolo.data = 'tunnel'

        elif self.global_index == 2 and self.mode == 'static0':
            to_yolo.data = 'static'

        # 정지선 15m 전부터 신호등 인식을 위한 YOLO 모델 활성화
        elif self.traffic_stop_wp <= 20:
            to_yolo.data = 'traffic'

        elif self.mode in ['delivery_A', 'delivery_B']:
            to_yolo.data = 'delivery'

        else:
            to_yolo.data = 'None'

        self.mode_pub.publish(to_yolo)


def main(args=None):

    # Initialise the node
    rclpy.init(args=args)

    try:
        # Initialise the class
        global_planner = GlobalPathPlanner()

        # Stop the node from exiting
        rclpy.spin(global_planner)

    finally:
        global_planner.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
