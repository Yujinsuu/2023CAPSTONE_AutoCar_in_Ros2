#!/usr/bin/env python3

import threading
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, String
from autocar_msgs.msg import Path2D, State2D, LinkArray
from ackermann_msgs.msg import AckermannDriveStamped

from autocar_nav import normalise_angle


class LowPassFilter:
    def __init__(self, cutoff_freq, update_rate):
        self.update_rate = update_rate
        self.alpha = cutoff_freq / (cutoff_freq + update_rate)
        self.filtered_angle = 0.0

    def update(self, input_angle):
        self.filtered_angle += self.alpha * (input_angle - self.filtered_angle)

        return self.filtered_angle


class PathTracker(Node):

    def __init__(self):

        super().__init__('path_tracker')

        # Initialise publishers
        self.tracker_pub = self.create_publisher(AckermannDriveStamped, '/autocar/ackermann_cmd', 10)
        self.ct_error_pub = self.create_publisher(Float64, '/autocar/cte_error', 10)
        self.h_error_pub = self.create_publisher(Float64, '/autocar/he_error', 10)

        # Initialise subscribers
        self.localisation_sub = self.create_subscription(State2D, '/autocar/state2D', self.vehicle_state_cb, 10)
        self.path_sub = self.create_subscription(Path2D, '/autocar/path', self.path_cb, 10)
        self.mission_status_sub = self.create_subscription(String, '/autocar/mission_status', self.mission_status_cb, 10)
        self.links_sub = self.create_subscription(LinkArray, '/autocar/mode', self.links_cb, 10)
        self.autocar_sub = self.create_subscription(AckermannDriveStamped, '/autocar/autocar_cmd', self.cmd_cb, 10)

        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', None),
                    ('steering_limits', None),
                    ('rearaxle_to_frontaxle', None),
                    ('centreofgravity_to_frontaxle', None)
                ]
            )

            self.frequency = float(self.get_parameter("update_frequency").value)
            self.max_steer = float(self.get_parameter("steering_limits").value)
            self.L = float(self.get_parameter("rearaxle_to_frontaxle").value)
            self.FL = float(self.get_parameter("centreofgravity_to_frontaxle").value)

        except ValueError:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class variables to use whenever within the class when necessary
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vel = 0.0

        self.cx = []
        self.cy = []
        self.cyaw = []

        self.target_idx = None
        self.heading_error = 0.0
        self.crosstrack_error = 0.0
        self.k = {'global'    : {'Straight': 1.0, 'Curve': 1.0},
                  'parking'   : {'Straight': 1.5, 'Curve': 1.5},
                  'revpark'   : {'Straight': 1.5, 'Curve': 1.5},
                  'uturn'     : {'Straight': 2.0, 'Curve': 2.0},
                  'static'    : {'Straight': 1.0, 'Curve': 1.0},
                  'dynamic'   : {'Straight': 1.0, 'Curve': 1.0},
                  'tollgate'  : {'Straight': 1.0, 'Curve': 1.0},
                  'tunnel'    : {'Straight': 1.0, 'Curve': 1.0},
                  'delivery_A': {'Straight': 1.0, 'Curve': 1.0},
                  'delivery_B': {'Straight': 1.0, 'Curve': 1.0},
                  'finish'    : {'Straight': 1.0, 'Curve': 1.0}}
        self.ksoft = 0.1
        self.kyaw = 1.0

        self.lock = threading.Lock()
        self.dt = 1 / self.frequency
        self.sigma = 0.0
        self.filter = LowPassFilter(cutoff_freq=4.3, update_rate=10.0)

        self.status = 'driving'
        self.mode = 'global'
        self.direction = 'Straight'

        # Intialise timers
        self.timer = self.create_timer(self.dt, self.stanley_control)

    def cmd_cb(self, msg):
        input_steer = msg.drive.steering_angle

        self.sigma = self.filter.update(input_steer)

    def vehicle_state_cb(self, msg):
        self.lock.acquire()

        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta
        self.vel = np.sqrt((msg.twist.x**2.0) + (msg.twist.y**2.0))
        if self.vel <= 1: self.vel = 1.0


        if self.cyaw:
            if (self.mode == 'parking' and self.status == 'return') or (self.mode == 'revpark' and self.status == 'parking'):
                d = -1
                self.x = self.x + d * np.cos(self.yaw)
                self.y = self.y + d * np.sin(self.yaw)

                d_yaw = d * np.tan(self.sigma) / self.L
                self.yaw = self.yaw + d_yaw

                self.target_index_calculator_backward()

            else:
                if self.mode == 'global' or self.vel > 10/3.6:
                    d = 2
                else:
                    d = 1
                self.x = self.x + d * np.cos(self.yaw)
                self.y = self.y + d * np.sin(self.yaw)

                d_yaw = d * np.tan(self.sigma) / self.L
                self.yaw = self.yaw + d_yaw

                self.target_index_calculator()

        self.lock.release()

    def path_cb(self, msg):
        self.lock.acquire()

        self.cx = []
        self.cy = []
        self.cyaw = []

        for i in range(0, len(msg.poses)):
            px = msg.poses[i].x
            py = msg.poses[i].y
            ptheta = msg.poses[i].theta
            self.cx.append(px)
            self.cy.append(py)
            self.cyaw.append(ptheta)

        self.lock.release()

    def mission_status_cb(self, msg):
        self.status = msg.data

    def links_cb(self, msg):
        self.mode = msg.mode
        self.direction = msg.direction


    def target_index_calculator(self):

        ''' Calculates the target index and each corresponding error '''

        # Calculate position of the front axle
        fx = self.x + self.FL * np.cos(self.yaw)
        fy = self.y + self.FL * np.sin(self.yaw)

        dx = [fx - icx for icx in self.cx] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in self.cy] # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy) # Find the distance from the front axle to the path
        target_idx = np.argmin(d) # Find the shortest distance in the array

        # Cross track error, project RMS error onto the front axle vector
        forward_vec = [np.sin(self.yaw), -np.cos(self.yaw)] # normal vector
        self.crosstrack_error = np.dot([dx[target_idx], dy[target_idx]], forward_vec)

        if self.mode != 'global':
            # Calculate position of the rear axle
            rx = self.x - (self.L - self.FL) * np.cos(self.yaw) # rear axle
            ry = self.y - (self.L - self.FL) * np.sin(self.yaw)

            dx = [rx - icx for icx in self.cx]
            dy = [ry - icy for icy in self.cy]

            d = np.hypot(dx, dy)
            target_idx = np.argmin(d)

        # Heading error
        self.heading_error = normalise_angle(self.cyaw[target_idx] - self.yaw)

        self.target_idx = target_idx

    def target_index_calculator_backward(self):

        ''' Calculates the target index and each corresponding error '''

        # Calculate position of the front axle of backward
        bx = self.x - (2 * self.L - self.FL) * np.cos(self.yaw) # 후진을 위한 가상의 front axle 설정
        by = self.y - (2 * self.L - self.FL) * np.sin(self.yaw)

        dx = [bx - icx for icx in self.cx[::-1]] # waypoints 순서를 반대로 뒤집기
        dy = [by - icy for icy in self.cy[::-1]]

        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Cross track error at front axle of backward
        backward_vec = [np.sin(self.yaw), -np.cos(self.yaw)] # normal vector
        self.crosstrack_error = np.dot([dx[target_idx], dy[target_idx]], backward_vec)

        # Calculate position of the rear axle
        rx = self.x - (self.L - self.FL) * np.cos(self.yaw) # rear axle
        ry = self.y - (self.L - self.FL) * np.sin(self.yaw)

        dx = [rx - icx for icx in self.cx[::-1]]
        dy = [ry - icy for icy in self.cy[::-1]]

        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Heading error
        self.heading_error = normalise_angle(self.yaw - self.cyaw[::-1][target_idx]) # waypoints 순서를 반대로 뒤집은 후, yaw과 비교
        self.target_idx = target_idx

    def stanley_control(self):

        self.lock.acquire()
        crosstrack_term = np.arctan2((self.k[self.mode][self.direction] * self.crosstrack_error), (self.ksoft + self.vel))
        heading_term = normalise_angle(self.kyaw * self.heading_error)

        cte = Float64()
        cte.data = np.rad2deg(crosstrack_term)
        if cte.data > 30: cte.data = 30.0
        elif cte.data < -30: cte.data = -30.0
        self.ct_error_pub.publish(cte)
        he = Float64()
        he.data = np.rad2deg(heading_term)
        if he.data > 30: he.data = 30.0
        elif he.data < -30: he.data = -30.0
        self.h_error_pub.publish(he)

        sigma_t = crosstrack_term + heading_term

        # Constrains steering angle to the vehicle limits
        if sigma_t >= self.max_steer:
            sigma_t = self.max_steer

        elif sigma_t <= -self.max_steer:
            sigma_t = -self.max_steer

        self.set_vehicle_command(sigma_t)
        self.lock.release()


    # Publishes to vehicle state
    def set_vehicle_command(self, steer):
        ''' Publishes the calculated steering angle  '''

        car = AckermannDriveStamped()
        car.header.frame_id = 'odom'
        car.header.stamp = self.get_clock().now().to_msg()

        car.drive.steering_angle = steer

        self.tracker_pub.publish(car)


def main(args=None):

    # Initialise the node
    rclpy.init(args=args)

    try:
        # Initialise the class
        path_tracker = PathTracker()

        # Stop the node from exiting
        rclpy.spin(path_tracker)

    finally:
        path_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
