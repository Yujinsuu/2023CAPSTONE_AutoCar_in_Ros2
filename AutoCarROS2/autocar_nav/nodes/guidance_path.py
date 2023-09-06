#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import copy
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Path
from autocar_msgs.msg import VisionSteer
from geometry_msgs.msg import PoseStamped, TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray

from autocar_nav import normalise_angle
from autocar_nav.quaternion import yaw_to_quaternion, euler_from_quaternion
from autocar_nav.transform_to_matrix import transform_to_matrix
from autocar_nav.delaunay_triangulation import DelaunayTriPath

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class LowPassFilter:
    def __init__(self, cutoff_freq, update_rate):
        self.update_rate = update_rate
        self.alpha = cutoff_freq / (cutoff_freq + update_rate)
        self.filtered_angle = 0.0

    def update(self, input_angle):
        self.filtered_angle += self.alpha * (input_angle - self.filtered_angle)

        return self.filtered_angle


class make_delaunay(Node):
    def __init__(self):
        super().__init__('guidance_path')

        qos_profile = QoSProfile(depth=10)
        self.sub_cluster = self.create_subscription(MarkerArray, '/markers', self.cluster_callback, qos_profile)
        self.autocar_sub = self.create_subscription(AckermannDriveStamped, '/autocar/autocar_cmd', self.cmd_callback, 10)

        self.cone_pub = self.create_publisher(MarkerArray, '/rviz/rubber_cone', 10)
        self.Local_path_pub = self.create_publisher(Path, '/rviz/track_path', 10)
        self.track_steer_pub = self.create_publisher(VisionSteer, '/autocar/track_steer', 10)

        self.L = 1.04
        self.cone_check = False

        self.cluster = MarkerArray()
        self.deltri = None
        self.path_x = None
        self.path_y = None
        self.path_yaw = None

        self.velocity = 1.5
        self.k = 1.0

        self.max_steer = 0.42
        self.sigma = 0.0
        self.filter = LowPassFilter(cutoff_freq=4.3, update_rate=10.0)

        self.id = 0

        self.world_frame = "odom"
        self.detection_frame = "car"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer1 = self.create_timer(0.1, self.delaunay_callback)
        self.timer2 = self.create_timer(0.1, self.stanley_callback)


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

        return float(result[0, 3]), float(result[1, 3]), yaw


    def cluster_callback(self, msg):
        self.cluster = msg
        virtual_cone = [[-1, -0.5, 0], [-1.5, -0.5, 0], [-2, -0.5, 0],
                        [-1,  0.5, 1], [-1.5,  0.5, 1], [-2,  0.5, 1]]

        points = []
        for n in range(len(virtual_cone)):
            points.append(virtual_cone[n])

        num = 0
        # 0_Blue : 차량 우측 (y < 0), 1_Yellow : 차량 좌측 (y > 0)
        for id, obs in enumerate(self.cluster.markers):
            xmin = obs.points[1].x
            xmax = obs.points[0].x
            ymin = obs.points[3].y
            ymax = obs.points[4].y

            depth = abs(xmax - xmin)
            width = abs(ymax - ymin)

            if depth <= 0.4 and width <= 0.4:
                x = (xmin + xmax) / 2
                y = (ymin + ymax) / 2

                # Yellow : y > mx + b
                m, b = -0.35, 0
                if (0 < x < 8 and m * x + b <= y < 1.5) or (-1 < x <= 0 and -0.6 <= y < 1.5): # 1_Yellow
                    point = [x, y, 1]
                    points.append(point)
                    num += 1

                    points.append([x-1, y-3,0])

                # elif y < -0.6   and -1.2 < x < 3: # 0_Blue
                #     point = [x, y, 0]
                #     points.append(point)

        if num < 3:
            self.cone_check = False
        else:
            self.cone_check = True

        self.deltri = DelaunayTriPath(np.array(points))

    def delaunay_callback(self):
        self.marker_array = MarkerArray()

        if self.deltri is not None:
            midpoints   = self.deltri.get_mid()
            blue_cone   = self.deltri.get_blue()
            yellow_cone = self.deltri.get_yellow()
            print(blue_cone, yellow_cone)

            self.create_marker(blue_cone, "blue")
            self.create_marker(yellow_cone, "yellow")

            self.create_marker(midpoints, "mid_point")
            self.create_marker([[-3, 0]]*30, "mid_point")
            self.id = 0

            self.make_path_curve_fit(midpoints) # 3차 함수 피팅

        self.cone_pub.publish(self.marker_array)

    def create_marker(self, xy, cone_color) :
        for i in range(len(xy)):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.id = self.id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = marker.SPHERE
            marker.action = marker.ADD

            odom_x, odom_y, odom_yaw = self.change_frame(float(xy[i][0]), float(xy[i][1]), 0.0, self.world_frame, self.detection_frame)
            marker.pose.position.x = odom_x
            marker.pose.position.y = odom_y
            marker.pose.position.z = 0.0

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25

            if cone_color == 'blue':
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0

            elif cone_color == 'yellow':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0

            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0

            self.id += 1
            self.marker_array.markers.append(marker)

    def curve_fit(self, midpoints): # midpoint 통해서 path 어떻게 만들 것 인지
        if isinstance(midpoints, list):
            midpoints = np.array(midpoints)

        x_data = midpoints[:, 0]
        y_data = midpoints[:, 1]

        # 3차 다항식
        coefficients = np.polyfit(x_data, y_data, deg=2)
        p = np.poly1d(coefficients)
        #print(x_data[0], x_data[-1])
        # 경로 생성
        path_x = np.arange(x_data[0], 6, 0.1) # 경로 길이제한 3m
        path_y = p(path_x)

        return path_x, path_y

    def make_path_curve_fit(self, midpoints):
        self.path_x, self.path_y = self.curve_fit(midpoints)
        num = len(self.path_x)

        self.path_yaw = []
        for i in range(num - 1):
            self.path_yaw.append(np.arctan2((self.path_y[i+1] - self.path_y[i]),(self.path_x[i+1] - self.path_x[i])))
        self.path_yaw.append(self.path_yaw[-1])

        viz_path = Path()
        viz_path.header.frame_id = "map"
        viz_path.header.stamp = self.get_clock().now().to_msg()

        for n in range(num):
            wx, wy, wyaw = self.change_frame(float(self.path_x[n]), float(self.path_y[n]), self.path_yaw[n], self.world_frame, self.detection_frame)

            vpose = PoseStamped()
            vpose.header.frame_id = "map"
            vpose.header.stamp = self.get_clock().now().to_msg()
            vpose.pose.position.x = wx
            vpose.pose.position.y = wy
            vpose.pose.position.z = 0.0
            vpose.pose.orientation = yaw_to_quaternion(np.pi * 0.5 - wyaw)
            viz_path.poses.append(vpose)

        self.Local_path_pub.publish(viz_path)


    def cmd_callback(self, msg):
        input_steer = msg.drive.steering_angle

        self.sigma = self.filter.update(input_steer)

    def stanley_callback(self):
        track = VisionSteer()

        car_x, car_y, car_yaw = 0,0,0

        if self.path_x is not None:
            dx = [car_x - icx for icx in self.path_x]
            dy = [car_y - icy for icy in self.path_y]

            d = np.hypot(dx, dy)
            idx = np.argmin(d)

            vector = [np.sin(car_yaw), -np.cos(car_yaw)]
            crosstrack_error = np.dot([dx[idx], dy[idx]], vector)
            crosstrack_term = np.arctan2((self.k * crosstrack_error), (self.velocity))

            dist = 1.5 * 0.425

            car_yaw = dist * np.tan(self.sigma) / self.L

            car_x = dist * np.cos(car_yaw)
            car_y = dist * np.sin(car_yaw)

            dx = [car_x - icx for icx in self.path_x]
            dy = [car_y - icy for icy in self.path_y]

            d = np.hypot(dx, dy)
            idx = np.argmin(d)

            heading_term = normalise_angle(self.path_yaw[idx] - car_yaw)

            sigma_t = crosstrack_term + heading_term

            # Constrains steering angle to the vehicle limits
            if sigma_t >= self.max_steer:
                sigma_t = self.max_steer

            elif sigma_t <= -self.max_steer:
                sigma_t = -self.max_steer

            track.steer = sigma_t
            track.detected = self.cone_check

            self.track_steer_pub.publish(track)


def main(args=None):
    rclpy.init(args=args)
    node = make_delaunay()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt')

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ =="__main__" :
    main()
