#!/usr/bin/env python3

import os
import time
import math
import numpy as np
import pandas as pd
from collections import deque
from tf2_ros import StaticTransformBroadcaster

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory
from sklearn.linear_model import RANSACRegressor

from std_msgs.msg import Float64MultiArray, Float32MultiArray, Float32
from nav_msgs.msg import Path, Odometry
from autocar_msgs.msg import State2D, LinkArray, Path2D
from geometry_msgs.msg import PoseStamped, TransformStamped

from autocar_nav.quaternion import yaw_to_quaternion


class Localization(Node):

    def __init__(self):

        super().__init__('localization')

        # Initialise publishers
        self.localization_pub = self.create_publisher(State2D, '/autocar/state2D', 10)
        self.trajectory_pub = self.create_publisher(Path, '/rviz/trajectory', 10)
        self.offset_pub = self.create_publisher(Float64MultiArray, '/autocar/dr_offset', 10)

        # Initialise subscribers
        self.GPS_odom_sub = self.create_subscription(Odometry, '/autocar/odom', self.vehicle_state_cb, 10)
        self.EKF_odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.dead_reckoning_cb, 10, callback_group=ReentrantCallbackGroup())
        self.mode_sub = self.create_subscription(LinkArray, '/autocar/mode', self.mode_cb, 10)
        self.pose_offset_sub= self.create_subscription(Float32MultiArray , '/data/key_offset', self.pose_offset_cb, 10)
        self.goals_sub = self.create_subscription(Path2D, '/autocar/goals', self.goals_cb, 10)
        self.lateral_error_sub = self.create_subscription(Float32, '/lanenet/lateral_error', self.lateral_error_cb, 10)


        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', 10.0),
                    ('centreofgravity_to_frontaxle', 1.04)
                ]
            )

            self.frequency = float(self.get_parameter("update_frequency").value)
            self.cg2frontaxle = float(self.get_parameter("centreofgravity_to_frontaxle").value)

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        file_path = os.path.join(get_package_share_directory('autocar_map'), 'data')
        df = pd.read_csv(file_path + '/kcity/tunnel_map.csv')
        self.tunnel_x = df['X'].tolist()
        self.tunnel_y = df['Y'].tolist()

        # Class constants
        self.state2d = None
        self.state = None

        self.dr_state2d = None
        self.dr_state = None
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.init_x = 0.0
        self.init_y = 0.0

        self.cov1 =0.0
        self.cov2 =0.0
        
        self.dx_key_offset = 0.0
        self.dy_key_offset = 0.0

        self.gp = []
        self.dp = []
        self.min_offset_list = []

        self.dx = 0.0
        self.dy = 0.0
        self.dgx = 0.0
        self.dgy = 0.0
        self.dDx = 0.0
        self.dDy = 0.0
        self.dr_mode = False
        self.get_offset = False

        self.GtoL = 1.29 # gps to lidar distance
        self.mode = 'global'
        self.waypoint = 0
        self.tunnel_exit = False
        self.odom_state = 'GPS Odometry'

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.ransac = RANSACRegressor()

        self.tx = deque([], 2000)
        self.ty = deque([], 2000)
        self.tw = deque([], 2000)

        self.ds = 1 / self.frequency
        self.timer = self.create_timer(self.ds, self.trajectory)
        
        self.lateral_error = 0.0
                
    def goals_cb(self, msg):
        self.ax = []
        self.ay = []
        for i in range(len(msg.poses)):
            px = msg.poses[i].x
            py = msg.poses[i].y
            self.ax.append(px)
            self.ay.append(py)

        self.path_yaw = self.get_path_yaw(self.ax, self.ay)
        
    def lateral_error_cb(self, msg):

        self.lateral_error =  msg.data
        #self.state2d.pose.theta
        #self.path_yaw
        
        
        
    def vehicle_state_cb(self, msg):
        zz = 3
        self.state = msg
        self.gp.append((msg.pose.pose.position.x,msg.pose.pose.position.y,time.time()))

        if len(self.gp) > 100:
            del self.gp[0]

        # self.get_logger().info('cov1  : %f' %msg.pose.covariance[0])
        # self.get_logger().info('cov2  : %f' %msg.pose.covariance[7])
        # self.get_logger().info('dr_mode  : %s' %self.dr_mode)
        # self.get_logger().info('zz  : %f' %zz)
        #cov

        self.cov1 =msg.pose.covariance[0]
        self.cov2 =msg.pose.covariance[7]
        # if msg.pose.covariance[0] > 0.2 or msg.pose.covariance[7] > 0.2:
        #     self.dr_mode = True

        if self.dr_state is not None:
            self.get_logger().info(self.odom_state)
            if self.dr_mode == True:
                self.update_state(self.dr_state)
            else:
                self.update_state(self.state)


    def dead_reckoning_cb(self, msg):

        # if (self.cov1 > 0.2 or self.cov2 > 0.2) or self.mode == 'tunnel':
        #     self.dr_mode = True
        #     self.odom_state = 'Dead-Reckoning'
        # elif (self.cov1 < 0.1 or self.cov2 < 0.1) and self.mode != 'tunnel':
        #     self.dr_mode = False
        #     self.odom_state = 'GPS-Odometry'


        self.dp.append((msg.pose.pose.position.x,msg.pose.pose.position.y,time.time()))
        if len(self.dp) > 100:
            del self.dp[0]
        #rtcm msg > rtcm callback에서

        #   self.dr_mode = True
        #self.get_logger().info('dr_mode  : %s' %self.dr_mode)

        if (self.dr_mode == True) and (self.get_offset == False):
            self.init_x = msg.pose.pose.position.x
            self.init_y = msg.pose.pose.position.y
            self.get_past_position()
            self.offset_x = self.dgx + (self.init_x - self.dDx)
            self.offset_y = self.dgy + (self.init_y - self.dDy)
            self.get_offset = True

        elif (self.dr_mode == True) and (self.get_offset == True) and (150 <= self.waypoint <= 152): # kcity
        # elif (self.dr_mode == True) and (self.get_offset == True) and (75 <= self.waypoint <= 80):
            if not self.tunnel_exit:
                index = self.get_lateral_error(self.dr_state.pose.pose.position.x, self.dr_state.pose.pose.position.y)
                self.offset_x = self.tunnel_x[index]
                self.offset_y = self.tunnel_y[index]
                self.init_x = msg.pose.pose.position.x
                self.init_y = msg.pose.pose.position.y
                self.tunnel_exit = True

        self.dr_state = msg
        self.dr_state.pose.pose.position.x = msg.pose.pose.position.x - self.init_x + self.offset_x + self.offset_x + self.dx_key_offset
        self.dr_state.pose.pose.position.y = msg.pose.pose.position.y - self.init_y + self.offset_y + self.offset_y + self.dy_key_offset

        offset = Float64MultiArray()
        offset.data = [- self.init_x + self.offset_x + self.dx_key_offset, - self.init_y + self.offset_y + self.dy_key_offset]
        self.offset_pub.publish(offset)
        #self.get_logger().info('offset  : %s' %offset.data)

        if len(self.gp) >20:
            #time_offset = abs(self.gp[-1][2] - self.dp[-1][2])
            gp_list = [item[2] for item in self.gp[-15:-1]]
            dp_list = [item[2] for item in self.dp[-15:-1]]
            dp_gp_offset = [abs(item[2]-gp_list[-14]) for item in self.dp[-15:-1]]
            dp_gp_offset_min = min(dp_gp_offset)
            self.min_offset_list.append(dp_gp_offset_min)
            max_min = max(self.min_offset_list)

            #debug and select dp_gp_offset_max
            # self.get_logger().info('gps_list  : %s' %(gp_list))
            # self.get_logger().info('DP_list : %s' %(dp_list))
            # self.get_logger().info('dp_gp_offset  : %s' %(dp_gp_offset))
            # self.get_logger().info('dp_gp_offset_max  : %s' %(max_min)) # 1.5sec - 0.72



        # if self.state is not None:
        #     if self.mode == 'tunnel':
        #         self.mode_change += 1


        #     if 1 <= self.mode_change <= 3:
        #         self.odom_state = 'Dead_Reckoning'
        #         self.offset_x = self.state.pose.pose.position.x
        #         self.offset_y = self.state.pose.pose.position.y
        #         self.init_x = msg.pose.pose.position.x
        #         self.init_y = msg.pose.pose.position.y

        #     self.dr_state = msg
        #     #offset filterd position
        #     self.dr_state.pose.pose.position.x = msg.pose.pose.position.x - self.init_x + self.offset_x
        #     self.dr_state.pose.pose.position.y = msg.pose.pose.position.y - self.init_y + self.offset_y

            # offset = Float64MultiArray()
            # offset.data = [- self.init_x + self.offset_x, - self.init_y + self.offset_y]
            # self.offset_pub.publish(offset)

    def get_past_position(self):
        found = False

        # Check the valid index range for gp and dp lists
        gp_length = len(self.gp)
        dp_length = len(self.dp)

        for i in range(-15, -gp_length, -1):
            for j in range(-1, -dp_length, -1):
                if abs(self.gp[i][2] - self.dp[j][2]) < 0.072:
                    self.dgx = self.gp[i][0]
                    self.dgy = self.gp[i][1]
                    self.dDx = self.dp[j][0]
                    self.dDy = self.dp[j][1]
                    found = True
            if found:
                break


    def get_lateral_error(self, x, y):
        wp_num = len(self.tunnel_x)

        dx = [x - icx for icx in self.tunnel_x]
        dy = [y - icy for icy in self.tunnel_y]

        d = np.hypot(dx, dy)
        closest_id = int(np.argmin(d))

        return closest_id


    def mode_cb(self, msg):
        self.mode = msg.mode
        self.waypoint = msg.closest_wp if self.mode == 'tunnel' else 0

    def status_cb(self, msg):
        self.status = msg.data

    def pose_offset_cb(self, msg):
        
        self.dx_key_offset = msg.data[2]
        self.dy_key_offset = msg.data[3]

    # Gets vehicle position from Gazebo and publishes data
    def update_state(self, state):
        # Define vehicle pose x,y, theta
        self.state2d = State2D()
        self.state2d.pose.x = state.pose.pose.position.x
        self.state2d.pose.y = state.pose.pose.position.y
        self.state2d.pose.theta = 2.0 * np.arctan2(state.pose.pose.orientation.z, state.pose.pose.orientation.w)

        # Aligning heading to y-axis, accounts for double rotation error
        if self.state2d.pose.theta < 0.0:
            self.state2d.pose.theta += 2.0 * np.pi

        # Define linear velocity x,y and angular velocity w
        self.state2d.twist.x = state.twist.twist.linear.x
        self.state2d.twist.y = state.twist.twist.linear.y
        self.state2d.twist.w = - state.twist.twist.angular.z

        self.localization_pub.publish(self.state2d)

        # create car frame
        transform = TransformStamped()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'car'
        transform.transform.translation.x = self.state2d.pose.x + self.GtoL * np.cos(self.state2d.pose.theta)
        transform.transform.translation.y = self.state2d.pose.y + self.GtoL * np.sin(self.state2d.pose.theta)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = np.sin(self.state2d.pose.theta / 2)
        transform.transform.rotation.w = np.cos(self.state2d.pose.theta / 2)

        # Broadcast the transform as a static transform
        self.tf_broadcaster.sendTransform(transform)

    def trajectory(self):
        if self.state2d != None:
            self.tx.append(self.state2d.pose.x + self.cg2frontaxle * np.cos(self.state2d.pose.theta))
            self.ty.append(self.state2d.pose.y + self.cg2frontaxle * np.sin(self.state2d.pose.theta))
            self.tw.append(self.state2d.pose.theta)

            if len(self.tx) > 2:
                # Path 메시지 구성
                path = Path()
                path.header.frame_id = "odom"
                path.header.stamp = self.get_clock().now().to_msg()

                path_length = min(len(self.tx), len(self.ty), len(self.tw))

                for n in range(path_length):
                    # Appending to Visualization Path
                    vpose = PoseStamped()
                    vpose.header.frame_id = "odom"
                    vpose.header.stamp = self.get_clock().now().to_msg()
                    vpose.pose.position.x = self.tx[n]
                    vpose.pose.position.y = self.ty[n]
                    vpose.pose.position.z = 0.0
                    vpose.pose.orientation = yaw_to_quaternion(self.tw[n] - np.pi * 0.5)
                    path.poses.append(vpose)

                self.trajectory_pub.publish(path)
    
    def get_path_yaw(self, ax ,ay):
        
        x = ax.reshape(-1, 1)
        y = ay.reshape(-1, 1)

        self.ransac.fit(x, y)

        slope = self.ransac.estimator_.coef_[0][0]
        yaw = math.atan2(slope, 1.0)

        return yaw
        


def main(args=None):

    # Initialise the node
    rclpy.init(args=args)

    try:
        # Initialise the class
        localization = Localization()

        # Stop the node from exiting
        rclpy.spin(localization)

    finally:
        localization.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
