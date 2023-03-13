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
        self.state2d = State2D()
        self.state = None
        self.tx = []
        self.ty = []
        self.tw = []
        self.timer = self.create_timer(3, self.trajectory)

    def vehicle_state_cb(self, msg):
        self.state = msg
        self.update_state()

    # Gets vehicle position from Gazebo and publishes data
    def update_state(self):
        # Define vehicle pose x,y, theta
        self.state2d = State2D()
        self.state2d.pose.x = self.state.pose.pose.position.x
        self.state2d.pose.y = self.state.pose.pose.position.y
        self.state2d.pose.theta = 2.0 * np.arctan2(self.state.pose.pose.orientation.z, self.state.pose.pose.orientation.w)

        # Aligning heading to y-axis, accounts for double rotation error
        if self.state2d.pose.theta < 0.0:
            self.state2d.pose.theta += 2.0 * np.pi

        # Define linear velocity x,y and angular velocity w
        self.state2d.twist.x = self.state.twist.twist.linear.x
        self.state2d.twist.y = self.state.twist.twist.linear.y
        self.state2d.twist.w = -self.state.twist.twist.angular.z

        self.localization_pub.publish(self.state2d)


    def trajectory(self):
        self.tx.append(self.state2d.pose.x)
        self.ty.append(self.state2d.pose.y)
        self.tw.append(self.state2d.pose.theta)

        if len(self.tx) > 2:
            # Path 메시지 구성
            path = Path()
            path.header.frame_id = "odom"
            path.header.stamp = self.get_clock().now().to_msg()

            path_length = min(len(self.tx), len(self.ty), len(self.tw))

            m = path_length - 2000 if len(self.tx) >= 2000 else 0

            for n in range(m, path_length):
                # Appending to Visualization Path
                vpose = PoseStamped()
                vpose.header.frame_id = "odom"
                vpose.header.stamp = self.get_clock().now().to_msg()
                vpose.pose.position.x = self.tx[n]
                vpose.pose.position.y = self.ty[n]
                vpose.pose.position.z = 0.0
                vpose.pose.orientation = yaw_to_quaternion(np.pi * 0.5 - self.tw[n])
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
