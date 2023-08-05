#!/usr/bin/env python3
#-*- coding: utf-8 -*-
import time
import numpy as np
from sklearn.linear_model import RANSACRegressor

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import LaserScan
from autocar_msgs.msg import Path2D
from geometry_msgs.msg import Point, Pose2D, TransformStamped
from visualization_msgs.msg import Marker

from autocar_nav.quaternion import euler_from_quaternion
from autocar_nav.transform_to_matrix import transform_to_matrix

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.create_rate(10)
        self.viz_pub = self.create_publisher(Marker, '/rviz/wall', 10)
        self.path_pub = self.create_publisher(Path2D, '/wall_path', 10)
        self.lane_pub = self.create_publisher(Path2D, '/wall_lane', 10)
        self.state_pub = self.create_publisher(String, '/tunnel_check', 10)

        self.scan_sub = self.create_subscription(LaserScan,'/scan', self.scan_callback, 10)
        self.mode_sub = self.create_subscription(String, '/yolo_mode', self.mode_callback, 10)

        # htech wall : 4.12m, kcity tunnel : 4.72m
        self.WalltoPath = 4.72
        self.LaneWidth = 3.6
        self.WalltoLane = [self.WalltoPath - 0.5 * self.LaneWidth, self.WalltoPath + 1.5 * self.LaneWidth]
        self.path_length = 30

        self.x_coords = None
        self.y_coords = None
        self.slope = None

        self.x_lane1 = None
        self.y_lane1 = None
        self.x_lane2 = None
        self.y_lane2 = None

        self.world_frame = "odom"
        self.detection_frame = "car"

        self.scan_data = None
        self.mode = 'None'
        self.state = String()
        self.state.data = 'entry'

        self.ransac = RANSACRegressor()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.scaned_publish)

    def mode_callback(self, msg):
        self.mode = msg.data

    def scan_callback(self, data):
        self.scan_data = data

        angle_min = self.scan_data.angle_min
        angle_max = self.scan_data.angle_max
        angle_inc = self.scan_data.angle_increment

        start_index = 50
        end_index = 450

        lidar_data = np.array(self.scan_data.ranges[start_index : end_index]) # 0- 897 0 - 450
        start_angle = angle_min + start_index * angle_inc

        angles = np.linspace(start_angle, start_angle + angle_inc * lidar_data.size, lidar_data.size)

        # x = r cos(theta), y = r sin(theta)
        x_coords_raw = np.multiply(lidar_data, np.cos(angles))
        y_coords_raw = np.multiply(lidar_data, np.sin(angles))

        wall_length = len(x_coords_raw)

        # inf 값의 인덱스 탐색 및 제거
        inf_indices = np.where(np.isinf(y_coords_raw))[0]
        x_coords_raw = np.delete(x_coords_raw, inf_indices)
        y_coords_raw = np.delete(y_coords_raw, inf_indices)

        self.tunnel_check(inf_indices, wall_length)

        # x raw 값의 범위 지정
        x_indices = np.where((x_coords_raw < -10) | (x_coords_raw > 30))[0]
        x_coords_raw = np.delete(x_coords_raw, x_indices)
        y_coords_raw = np.delete(y_coords_raw, x_indices)
        x_coords, y_coords, x_lane1, y_lane1, x_lane2, y_lane2 = self.get_line_RANSAC(x_coords_raw, y_coords_raw)
        self.slope = np.arctan2(y_coords[-1]-y_coords[0],x_coords[-1]-x_coords[0])

        self.x_coords = x_coords
        self.y_coords = y_coords
        self.x_lane1 = x_lane1
        self.y_lane1 = y_lane1
        self.x_lane2 = x_lane2
        self.y_lane2 = y_lane2


    def tunnel_check(self, inf_index, scan_range):
        if self.mode == 'tunnel':
            f_inf = len(inf_index[inf_index > 0.6 * scan_range])
            r_inf = len(inf_index[inf_index < 0.4 * scan_range])
            threshold = 0.2 * scan_range

            if self.state.data == 'entry':
                if (f_inf <= threshold <= r_inf) or (f_inf < threshold and r_inf < threshold):
                    self.state.data = 'inside'
            elif self.state.data == 'inside':
                if (f_inf >= threshold >= r_inf) or (f_inf > threshold and r_inf > threshold):
                    self.state.data = 'exit'


    def get_line_RANSAC(self, x_coords_raw, y_coords_raw) :

        X = x_coords_raw.reshape(-1, 1)
        y = y_coords_raw.reshape(-1, 1)

        self.ransac.fit(X, y)

        slope = self.ransac.estimator_.coef_[0][0]
        intercept = self.ransac.estimator_.intercept_[0]

        x_coords = np.arange(-10, 20)[:, np.newaxis]
        # num = min(self.path_length, len(x_coords))
        # x_coords = x_coords[:num]
        y_coords = self.ransac.predict(x_coords)

        x_lane1 = x_coords
        x_lane2 = x_coords
        y_lane1 = y_coords
        y_lane2 = y_coords

        self.plot_line(x_coords, y_coords)

        if abs(slope) < 1e6:
            y_coords += self.WalltoPath
            y_lane1 += self.WalltoLane[0]
            y_lane2 += self.WalltoLane[1]
        elif abs(slope) > 2000000000:
            x_coords -= self.WalltoPath
            x_lane1 -= self.WalltoLane[0]
            x_lane2 -= self.WalltoLane[1]
        else:
            translation_vector = self.WalltoPath * np.array([-1 / slope, 1])
            t1_vecotr = self.WalltoLane[0] * np.array([-1 / slope, 1])
            t2_vecotr = self.WalltoLane[1] * np.array([-1 / slope, 1])
            moved_coords = np.column_stack((x_coords, y_coords)) - translation_vector
            m1_coords = np.column_stack((x_lane1, y_lane1)) - t1_vecotr
            m2_coords = np.column_stack((x_lane2, y_lane2)) - t2_vecotr
            x_coords = moved_coords[:, 0]
            y_coords = moved_coords[:, 1]
            x_lane1 = m1_coords[:, 0]
            y_lane1 = m1_coords[:, 1]
            x_lane2 = m2_coords[:, 0]
            y_lane2 = m2_coords[:, 1]

        return x_coords, y_coords, x_lane1, y_lane1, x_lane2, y_lane2

    def plot_line(self, x_coords, y_coords):
        line_marker = Marker()
        line_marker.header.frame_id = "car"
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.1
        line_marker.scale.y = 0.1
        line_marker.scale.z = 0.1
        line_marker.color.a = 1.0
        line_marker.color.r = 0.0
        line_marker.color.g = 0.0
        line_marker.color.b = 1.0

        for i in range(len(x_coords)):
            point = Point()
            point.x = float(x_coords[i])
            point.y = float(y_coords[i])
            point.z = 0.0
            line_marker.points.append(point)

        self.viz_pub.publish(line_marker)


    def change_frame(self, x, y, yaw, world_frame, detection_frame):
        pose = np.array([[1.0, 0.0, 0.0, x],
                         [0.0, 1.0, 0.0, y],
                         [0.0, 0.0, 1.0, 0.0],
                         [0.0, 0.0, 0.0, 1.0]])

        try:
            now = rclpy.time.Time()
            t = self.tf_buffer.lookup_transform(world_frame, detection_frame, now)# Duration(seconds=1))
        except:
            self.get_logger().info("can't transform")
            t = TransformStamped()

        tf_matrix = transform_to_matrix(t)
        yaw = euler_from_quaternion(t.transform.rotation.x,
                                    t.transform.rotation.y,
                                    t.transform.rotation.z,
                                    t.transform.rotation.w)

        result = np.array(np.dot(tf_matrix, pose))
        # euler = tf.transformations.euler_from_matrix(result)


        return float(result[0, 3]), float(result[1, 3]), yaw  #, euler[2]


    def scaned_publish(self):
        wall_path = Path2D()
        wall_lane = Path2D()
        if self.x_coords is not None:
            waypoints = min(len(self.x_coords), len(self.y_coords))

            for i in range(min(waypoints, self.path_length)) :
                wx, wy, wyaw = self.change_frame(float(self.x_coords[i]), float(self.y_coords[i]), self.slope, self.world_frame, self.detection_frame)
                l1x, l1y, l1yaw = self.change_frame(float(self.x_lane1[i]), float(self.y_lane1[i]), self.slope, self.world_frame, self.detection_frame)
                l2x, l2y, l2yaw = self.change_frame(float(self.x_lane2[i]), float(self.y_lane2[i]), self.slope, self.world_frame, self.detection_frame)

                path = Pose2D()
                path.x = wx
                path.y = wy

                wall_path.poses.append(path)

                lane1 = Pose2D()
                lane1.x = l1x
                lane1.y = l1y

                wall_lane.poses.append(lane1)

                lane2 = Pose2D()
                lane2.x = l2x
                lane2.y = l2y

                wall_lane.poses.append(lane2)

        self.path_pub.publish(wall_path)
        self.state_pub.publish(self.state)


def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()

    try :
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        pass
    finally :
        rclpy.shutdown()

if __name__ == '__main__':
    main()
