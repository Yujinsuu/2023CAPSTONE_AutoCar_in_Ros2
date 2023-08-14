#!/usr/bin/env python3

import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, String
from nav_msgs.msg import Path, Odometry
from autocar_msgs.msg import State2D, LinkArray
from geometry_msgs.msg import PoseStamped

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
        self.EKF_odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.dead_reckoning_cb, 10)
        self.mode_sub = self.create_subscription(LinkArray, '/autocar/mode', self.mode_cb, 10)
        self.status_sub = self.create_subscription(String , '/autocar/mission_status', self.status_cb, 10)

        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', None),
                    ('centreofgravity_to_frontaxle', None)
                ]
            )

            self.frequency = float(self.get_parameter("update_frequency").value)
            self.cg2frontaxle = float(self.get_parameter("centreofgravity_to_frontaxle").value)

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class constants
        self.state2d = None
        self.state = None

        self.dr_state2d = None
        self.dr_state = None
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.init_x = 0.0
        self.init_y = 0.0

        self.mode = 'global'
        self.status = 'driving'
        self.mode_change = 0
        self.odom_state = 'GPS Odometry'

        self.tx = deque([], 2000)
        self.ty = deque([], 2000)
        self.tw = deque([], 2000)

        self.ds = 1 / self.frequency
        self.timer = self.create_timer(self.ds, self.trajectory)

    def vehicle_state_cb(self, msg):
        self.state = msg

        if self.dr_state is not None:
            if self.mode == 'tunnel':
                self.update_state(self.dr_state)
            else:
                self.odom_state = 'GPS_Odometry'
                self.update_state(self.state)


    def dead_reckoning_cb(self, msg):
        if self.state is not None:
            if self.mode != 'tunnel':
                self.offset_x = self.state.pose.pose.position.x
                self.offset_y = self.state.pose.pose.position.y
                self.init_x = msg.pose.pose.position.x
                self.init_y = msg.pose.pose.position.y

            self.dr_state = msg
            #offset filterd position
            self.dr_state.pose.pose.position.x = msg.pose.pose.position.x - self.init_x + self.offset_x
            self.dr_state.pose.pose.position.y = msg.pose.pose.position.y - self.init_y + self.offset_y

            offset = Float64MultiArray()
            offset.data = [- self.init_x + self.offset_x, - self.init_y + self.offset_y]
            self.offset_pub.publish(offset)


    def mode_cb(self, msg):
        self.mode = msg.mode

    def status_cb(self, msg):
        self.status = msg.data

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
