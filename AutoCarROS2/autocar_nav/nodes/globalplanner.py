#!/usr/bin/env python3

import os
import sys
import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Pose2D, PoseArray
from autocar_msgs.msg import Path2D, State2D

path_module = os.path.join(get_package_share_directory('autocar_map'), 'path')
sys.path.append(path_module)
from path_map import *

class GlobalPathPlanner(Node):

    def __init__(self):

        ''' Class constructor to initialise the class '''

        super().__init__('global_planner')

        # Initialise publisher(s)
        self.goals_pub = self.create_publisher(Path2D, '/autocar/goals', 10)
        # self.goals_viz_pub = self.create_publisher(PoseArray, '/autocar/viz_goals', 10)
        # Initialise suscriber(s)
        self.localisation_sub = self.create_subscription(State2D, '/autocar/state2D', self.vehicle_state_cb, 10)

        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('waypoints_ahead', None),
                    ('waypoints_behind', None),
                    ('passed_threshold', None),
                    ('waypoints', None),
                    ('centreofgravity_to_frontaxle', None)
                ]
            )

            self.wp_ahead = int(self.get_parameter("waypoints_ahead").value)
            self.wp_behind = int(self.get_parameter("waypoints_behind").value)
            self.passed_threshold = float(self.get_parameter("passed_threshold").value)
            self.cg2frontaxle = float(self.get_parameter("centreofgravity_to_frontaxle").value)

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Import waypoints.csv into class variables ax and ay
        mx = use_map.ax['global']
        my = use_map.ay['global']
        mw = use_map.aw['global']

        self.mx = mx[0:len(mx):1]
        self.my = my[0:len(my):1]
        self.mw = mw[0:len(mw):1]

        # Class constants
        self.wp_num = min(len(self.mx), len(self.my))
        self.wp_published = self.wp_ahead + self.wp_behind

        # Class variables to use whenever within the class when necessary
        self.x = None
        self.y = None
        self.theta = None

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

    def set_waypoints(self):
        '''
            Determines the appropriate set of waypoints to publish by the following steps

            1. Identify waypoint closest to front axle
            2. Determines if this point is ahead or behind, by transformation
            3. Preserves fixed number of points ahead or behind

            Parameters:
                self.wp_ahead           - Indicates number of waypoints to look ahead
                self.wp_behind          - Indicates number of waypoints to preserve behind the vehicle
                self.wp_published       - Indicates the total number of waypoints published
                self.passed_threshold   - Indicates the distance after which a waypoint is considered passed
                self.wp_num             - Total number of waypoints
        '''

        # Identify position of vehicle front axle
        fx = self.x + self.cg2frontaxle * -np.sin(self.theta)
        fy = self.y + self.cg2frontaxle * np.cos(self.theta)

        dx = [fx - icx for icx in self.mx] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in self.my] # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy)        # Find the distance from the front axle to the path
        closest_id = np.argmin(d)   # Returns the index with the shortest distance in the array

        transform = self.frame_transform(self.mx[closest_id], self.my[closest_id], fx, fy, self.theta)

        if closest_id < 2:
            # If the vehicle is starting along the path
            self.get_logger().info('Closest Waypoint #{} (Starting Path)'.format(closest_id))
            px = self.mx[0: self.wp_published]
            py = self.my[0: self.wp_published]
            pw = self.my[0: self.wp_published]

        elif closest_id > (self.wp_num - self.wp_published):
            # If the vehicle is finishing the given set of waypoints
            self.get_logger().info('Closest Waypoint #{} (Terminating Path)'.format(closest_id))
            px = self.mx[-self.wp_published:]
            py = self.my[-self.wp_published:]
            pw = self.my[-self.wp_published:]

        elif transform[1] < (0.0 - self.passed_threshold):
            # If the vehicle has passed, closest point is preserved as a point behind the car
            self.get_logger().info('Closest Waypoint #{} (Passed)'.format(closest_id))
            px = self.mx[closest_id - (self.wp_behind - 1) : closest_id + (self.wp_ahead + 1)]
            py = self.my[closest_id - (self.wp_behind - 1) : closest_id + (self.wp_ahead + 1)]
            pw = self.my[closest_id - (self.wp_behind - 1) : closest_id + (self.wp_ahead + 1)]

        else:
            # If the vehicle has yet to pass, a point behind the closest is preserved as a point behind the car
            self.get_logger().info('Closest Waypoint #{} (Approaching)'.format(closest_id))
            px = self.mx[(closest_id - self.wp_behind) : (closest_id + self.wp_ahead)]
            py = self.my[(closest_id - self.wp_behind) : (closest_id + self.wp_ahead)]
            pw = self.mw[(closest_id - self.wp_behind) : (closest_id + self.wp_ahead)]

        self.publish_goals(px, py, pw)

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

    def publish_goals(self, px, py, pw):

        ''' Publishes an array of waypoints for the Local Path Planner '''

        waypoints = min(len(px), len(py), len(pw))
        goals = Path2D()

        for i in range(0, waypoints):
            # Appending to Target Goals
            goal = Pose2D()
            goal.x = px[i]
            goal.y = py[i]
            goal.theta = pw[i]

            goals.poses.append(goal)

        self.goals_pub.publish(goals)

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
