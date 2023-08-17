#!/usr/bin/env python3

import os
import math
import numpy as np
import pandas as pd
from collections import deque

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Path, Odometry
from autocar_msgs.msg import State2D
from geometry_msgs.msg import PoseStamped, TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray

from autocar_nav.quaternion import yaw_to_quaternion

from tf2_ros import StaticTransformBroadcaster


class LowPassFilter:
    def __init__(self, cutoff_freq, update_rate):
        self.update_rate = update_rate
        self.alpha = cutoff_freq / (cutoff_freq + update_rate)
        self.filtered_angle = 0.0

    def update(self, input_angle):
        self.filtered_angle += self.alpha * (input_angle - self.filtered_angle)

        return self.filtered_angle

class Simulation(Node):

    def __init__(self):

        super().__init__('simulation')

        # Initialise publishers
        self.localization_pub = self.create_publisher(State2D, '/autocar/state2D', 10)
        self.trajectory_pub = self.create_publisher(Path, '/rviz/trajectory', 10)
        self.Car_pub = self.create_publisher(MarkerArray, "/rviz/gps_odometry_marker", 10)
        self.viz_steer = self.create_publisher(Marker, '/rviz/viz_steer', 10)

        # Initialise subscribers
        self.cmd_sub = self.create_subscription(AckermannDriveStamped, '/autocar/autocar_cmd', self.vehicle_cb, 10)

        # Class constants
        self.x = None
        self.y = None
        self.theta = None
        self.sigma = 0.0
        self.vel = 0.0
        self.filter = LowPassFilter(cutoff_freq=4.3, update_rate=10.0)

        file_path = os.path.join(get_package_share_directory('autocar_map'), 'data')
        df = pd.read_csv(file_path + '/htech/park_test.csv')
        map_x = df[df['Link']==0]['X-axis'].to_list()[0:2]
        map_y = df[df['Link']==0]['Y-axis'].to_list()[0:2]
        self.init_x = map_x[0]
        self.init_y = map_y[0]
        self.init_yaw = np.arctan2((map_y[1] - self.init_y), (map_x[1] - self.init_x))


        self.state2d = None
        self.state = Odometry()
        self.tx = deque([], 2000)
        self.ty = deque([], 2000)
        self.tw = deque([], 2000)

        self.L = 1.04

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.trajectory)

    def calculate_next_position(self, x0, y0, yaw, sigma, v):
        d = v * self.dt
        x1 = x0 + d * np.cos(yaw)
        y1 = y0 + d * np.sin(yaw)

        d_sigma = sigma - self.sigma
        if abs(d_sigma) > 0.1:
            if sigma > self.sigma:
                self.sigma += 0.1
            elif sigma < self.sigma:
                self.sigma -= 0.1
        else:
            self.sigma += d_sigma

        d_yaw = d * np.tan(self.sigma) / self.L
        new_yaw = yaw + d_yaw

        return x1, y1, new_yaw

    def vehicle_cb(self, msg):
        if self.x == None:
            self.x = self.init_x
            self.y = self.init_y
            self.theta = self.init_yaw

        input_steer = msg.drive.steering_angle
        sigma = self.filter.update(input_steer)
        self.vel = msg.drive.speed

        rev = -1 if msg.drive.acceleration == 2.0 else 1
        car_speed = rev * self.vel

        if self.x != None:
            self.x, self.y, self.theta = self.calculate_next_position(self.x, self.y, self.theta, sigma, car_speed)

            self.state.pose.pose.position.x = self.x
            self.state.pose.pose.position.y = self.y
            self.state.pose.pose.orientation.z = np.sin(self.theta / 2)
            self.state.pose.pose.orientation.w = np.cos(self.theta / 2)
            self.state.twist.twist.linear.x = self.vel * np.cos(self.theta)
            self.state.twist.twist.linear.y = self.vel * np.sin(self.theta)

            self.update_state()
            self.visual_steer()

            car =MarkerArray()
            m1 = self.visual_car()
            car.markers.append(m1)
            m2 = self.visual_yaw()
            car.markers.append(m2)

            self.Car_pub.publish(car)


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

        self.localization_pub.publish(self.state2d)


    def trajectory(self):
        if self.state2d != None:
            self.tx.append(self.state2d.pose.x + self.L/2 * np.cos(self.state2d.pose.theta))
            self.ty.append(self.state2d.pose.y + self.L/2 * np.sin(self.state2d.pose.theta))
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

    def visual_car(self):
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = self.get_clock().now().to_msg()
        m.id = 1
        m.type = m.CUBE
        m.pose.position.x = self.x + (self.L / 2) * math.cos(self.theta)
        m.pose.position.y = self.y + (self.L / 2) * math.sin(self.theta)
        m.pose.position.z = 0.45

        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = np.sin(self.theta / 2)
        m.pose.orientation.w = np.cos(self.theta / 2)

        m.scale.x = 1.600
        m.scale.y = 1.160
        m.scale.z = 1.000
        m.color.r = 17 / 255.0
        m.color.g = 17 / 255.0
        m.color.b = 252 / 255.0
        m.color.a = 0.97

        return m

    def visual_yaw(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.id = 2
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = self.x + (self.L / 2) * math.cos(self.theta)
        m.pose.position.y = self.y + (self.L / 2) * math.sin(self.theta)
        m.pose.position.z = 0.3
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = np.sin(self.theta/2)
        m.pose.orientation.w = np.cos(self.theta/2)
        m.scale.x = 2.
        m.scale.y = 0.3
        m.scale.z = 0.1
        m.color.r = 17 / 255.0
        m.color.g = 17 / 255.0
        m.color.b = 252 / 255.0
        m.color.a = 0.97

        return m

    def visual_steer(self):
        steer = self.sigma
        theta = self.theta + steer

        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'arrows'
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.x + (self.L / 2) * math.cos(self.theta)
        marker.pose.position.y = self.y + (self.L / 2) * math.sin(self.theta)
        marker.pose.position.z = 0.6
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = np.sin(theta/2)
        marker.pose.orientation.w = np.cos(theta/2)
        marker.scale.x = 2.
        marker.scale.y = 0.3
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.viz_steer.publish(marker)

def main(args=None):

    # Initialise the node
    rclpy.init(args=args)
    node = Simulation()

    # create odom frame
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'odom'

    # Broadcast the transform as a static transform
    static_broadcaster = StaticTransformBroadcaster(node)
    static_broadcaster.sendTransform(transform)

    try:
      rclpy.spin(node)
    except KeyboardInterrupt:
      node.get_logger().info('Keyboard Interrupt')
    finally:
      node.destroy_node()
      rclpy.shutdown()

if __name__=="__main__":
    main()
