#!/usr/bin/env python3

import os
import csv
import pandas as pd
import numpy as np
import math

import rclpy
from rclpy.node import Node
from autocar_msgs.msg import State2D
from std_msgs.msg import Float32
from sklearn.linear_model import RANSACRegressor
from ament_index_python.packages import get_package_share_directory
from autocar_nav.normalise_angle import normalise_angle


class Get_Yaw_Init(Node):
    def __init__(self):
        super().__init__('get_yaw_init')

        file_path = os.path.join(get_package_share_directory('autocar_map'), 'data')
        df = pd.read_csv(file_path + '/kcity/tunnel_map.csv')
        x = df['X'].to_numpy()
        y = df['Y'].to_numpy()


        # print("x 변수 데이터:", x)
        # print("y 변수 데이터:", y)

        # 선형보간, yaw 획득
        x = x.reshape(-1, 1)
        y = y.reshape(-1, 1)
        ransac = RANSACRegressor()
        ransac.fit(x, y)
        gradient = ransac.estimator_.coef_[0][0]
        
        self.map_yaw = math.atan2(gradient, 1.0)
        self.car_yaw = 0.0
        self.lidar_yaw = 0.0
        self.yaw_error = 0.0

        # wall fallower 에서 yaw값 받아서 비교
        # car yaw + slope from wall fallower

        self.car_yaw_sub= self.create_subscription(State2D , '/autocar/state2D', self.car_yaw_cb, 10)
        self.lidar_yaw_sub = self.create_subscription(Float32, '/lidar_yaw', self.lidar_yaw_cb,10)
    def car_yaw_cb(self, msg):
        
        self.car_yaw = msg.pose.theta
        
        self.yaw_error = normalise_angle(self.map_yaw - (self.lidar_yaw + self.car_yaw))
        print('yaw_error', self.yaw_error/math.pi*180)
        
    def lidar_yaw_cb(self, msg):
        
        self.lidar_yaw = msg.data



def main(args=None):
    rclpy.init(args=args)
    wall_follower = Get_Yaw_Init()

    try :
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        pass
    finally :
        rclpy.shutdown()

if __name__ == '__main__':
    main()
