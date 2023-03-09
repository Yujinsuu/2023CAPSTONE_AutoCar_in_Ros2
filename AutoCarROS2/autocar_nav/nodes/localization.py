#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from autocar_msgs.msg import State2D
from scipy.interpolate import CubicSpline, interp1d
from autocar_nav import generate_cubic_path, yaw_to_quaternion


class Localization(Node):

    def __init__(self):

        super().__init__('localization')

        # Initialise publishers
        self.localization_pub = self.create_publisher(State2D, '/autocar/state2D', 10)
        self.trajectory_pub = self.create_publisher(Path, '/rviz/trajectory', 10)

        # Initialise subscribers
        self.odom_sub = self.create_subscription(Odometry, '/autocar/odom', self.vehicle_state_cb, 10)

        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', None)
                ]
            )

            self.frequency = float(self.get_parameter("update_frequency").value)

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class constants
        self.state = None
        self.tx = []
        self.ty = []

    def vehicle_state_cb(self, msg):
        self.state = msg
        self.update_state()

    # Gets vehicle position from Gazebo and publishes data
    def update_state(self):

        # Define vehicle pose x,y, theta
        state2d = State2D()
        state2d.pose.x = self.state.pose.pose.position.x
        state2d.pose.y = self.state.pose.pose.position.y
        state2d.pose.theta = 2.0 * np.arctan2(self.state.pose.pose.orientation.z, self.state.pose.pose.orientation.w)

        # Aligning heading to y-axis, accounts for double rotation error
        if state2d.pose.theta < 0.0:
            state2d.pose.theta += 2.0 * np.pi

        # Define linear velocity x,y and angular velocity w
        state2d.twist.x = self.state.twist.twist.linear.x
        state2d.twist.y = self.state.twist.twist.linear.y
        state2d.twist.w = -self.state.twist.twist.angular.z

        self.localization_pub.publish(state2d)
        self.timer = self.create_timer(3, self.trajectory)

    def trajectory(self):
        self.tx.append(self.state.pose.pose.position.x)
        self.ty.append(self.state.pose.pose.position.y)

        if len(self.tx) > 2:
            f = interp1d(self.tx, self.ty, kind='linear')

            # 보간 구간 설정 (예: 0.1초 간격으로 보간)
            t = np.linspace(self.tx[0], self.tx[-1], num=100)
            interp_y = f(t)

            # Path 메시지 구성
            path = Path()
            path.header.frame_id = "odom"

            for j in range(len(t)):
                p = PoseStamped()
                p.pose.position.x = t[j]
                p.pose.position.y = interp_y[j]
                p.pose.orientation.w = 1.0
                path.poses.append(p)

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
