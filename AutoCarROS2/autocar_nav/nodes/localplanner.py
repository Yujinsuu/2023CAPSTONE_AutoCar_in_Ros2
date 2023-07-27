#!/usr/bin/env python3

import os
import numpy as np
import pandas as pd
from scipy.interpolate import CubicSpline

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64MultiArray
from autocar_msgs.msg import Path2D, State2D, ObjectArray, LinkArray
from geometry_msgs.msg import Pose2D, PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

from autocar_nav.quaternion import yaw_to_quaternion
from autocar_nav.hybrid_a_star import hybrid_a_star
from autocar_nav.separation_axis_theorem import separating_axis_theorem, get_vertice_rect

from tf2_ros import StaticTransformBroadcaster


class LocalPathPlanner(Node):

    def __init__(self):
        super().__init__('local_planner')

        # Initialise publishers
        self.local_planner_pub = self.create_publisher(Path2D, '/autocar/path', 10)
        self.path_viz_pub = self.create_publisher(Path, '/autocar/viz_path', 10)
        self.estop_pub = self.create_publisher(Bool, '/autocar/estop', 10)
        self.center_viz_pub = self.create_publisher(MarkerArray, '/rviz/pathlane', 10)

        # Initialise subscribers
        self.goals_sub = self.create_subscription(Path2D, '/autocar/goals', self.goals_cb, 10)
        self.localisation_sub = self.create_subscription(State2D, '/autocar/state2D', self.vehicle_state_cb, 10)
        self.offset_sub = self.create_subscription(Float64MultiArray, '/autocar/tunnel_offset', self.offset_cb, 10)
        self.obstacle_sub = self.create_subscription(ObjectArray, '/obstacles', self.obstacle_cb, 10)
        self.mode_sub = self.create_subscription(LinkArray, '/autocar/mode', self.mode_cb, 10)

        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', None),
                    ('frame_id', None),
                    ('car_width', None),
                    ('centreofgravity_to_frontaxle', None)
                ]
            )

            self.frequency = float(self.get_parameter("update_frequency").value)
            self.car_width = float(self.get_parameter("car_width").value)
            self.cg2frontaxle = float(self.get_parameter("centreofgravity_to_frontaxle").value)

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        ##################### 회피주행시 넘으면 안되는 선의 위치정보 ##########################
        file_path = os.path.join(get_package_share_directory('autocar_map'), 'data')
        df = pd.read_csv(file_path + '/htech/tunnel_lane.csv')
        self.center_x = df['x'].tolist()
        self.center_y = df['y'].tolist()
        self.center_yaw = df['yaw'].tolist()
        self.offset_x = 0.0
        self.offset_y = 0.0
        ###############################################################################
        self.ds = 1 / self.frequency

        self.ax = []
        self.ay = []
        self.obstacles = []

        self.target_path = Path2D()
        self.viz_path = Path()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.mode = 'global'
        self.GtoL = 1.29 # gps to lidar distance
        self.L = 1.3 # 차량 길이
        self.W = 0.9 # 차량 폭
        self.estop = False
        self.is_fail = False

        self.timer = self.create_timer(0.1, self.timer_cb)

        self.tf_broadcaster = StaticTransformBroadcaster(self)

    def timer_cb(self):
        # create car frame
        transform = TransformStamped()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'car'
        transform.transform.translation.x = self.x + self.GtoL * np.cos(self.yaw)
        transform.transform.translation.y = self.y + self.GtoL * np.sin(self.yaw)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = np.sin(self.yaw / 2)
        transform.transform.rotation.w = np.cos(self.yaw / 2)

        # Broadcast the transform as a static transform
        self.tf_broadcaster.sendTransform(transform)

        self.find_path()

    def goals_cb(self, msg):
        '''
        Callback function to recieve immediate goals from global planner in global frame
        '''
        self.ax = []
        self.ay = []
        for i in range(len(msg.poses)):
            px = msg.poses[i].x
            py = msg.poses[i].y
            self.ax.append(px)
            self.ay.append(py)


    def vehicle_state_cb(self, msg):
        '''
        Callback function to recieve vehicle state information from localization in global frame
        '''
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta

    def mode_cb(self, msg):
        self.waypoint = msg.closest_wp
        self.mode = msg.mode
        self.traffic_stop_wp = msg.traffic_stop_wp
        self.parking_stop_wp = msg.parking_stop_wp
        self.direction = msg.direction
        self.next_path = msg.next_path

    def offset_cb(self, msg):
        self.offset_x, self.offset_y = msg.data

    def obstacle_cb(self, msg):
        self.obstacles = [(o.x, o.y, o.yaw, o.length, o.width) for o in msg.object_list]
        # 차선정보 넣기
        if self.mode in ['tunnel', 'static', 'dynamic']:
            path_lane = []
            for i in range(len(self.center_x)):
                # length는 waypoint 간격만큼, width는 차선의 폭 (가능하면 최대한 작게)
                path_lane.append((self.center_x[i] - self.offset_x, self.center_y[i] - self.offset_y, self.center_yaw[i], 2.0, 1.0))
            self.obstacles += path_lane

        self.viz_path_lane()


    def viz_path_lane(self):

        marray = MarkerArray()
        for i in range(len(self.center_x)):
            m = Marker()
            m.header.frame_id = "/map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.type = m.CUBE
            m.id = i

            m.pose.position.x = self.center_x[i] - self.offset_x
            m.pose.position.y = self.center_y[i] - self.offset_y
            m.pose.position.z = 0.75
            quat = yaw_to_quaternion(self.center_yaw[i])
            m.pose.orientation.x = quat.x
            m.pose.orientation.y = quat.y
            m.pose.orientation.z = quat.z
            m.pose.orientation.w = quat.w

            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 1.645

            m.color.r = 121 / 255.0
            m.color.g = 204 / 255.0
            m.color.b = 102 / 255.0
            m.color.a = 0.97

            m.lifetime = Duration(nanoseconds=150000000).to_msg()
            marray.markers.append(m)

        self.center_viz_pub.publish(marray)

    def determine_path(self, cx, cy, cyaw):
        self.estop = False
        obstacle_colliding = []
        car_msg = []
        for i in range(0,len(cyaw),10):
            car_msg.append((cx[i],cy[i],cyaw[i],self.L,self.W))

        for obs in self.obstacles:
            for car in car_msg:
                car_vertices = get_vertice_rect(car)
                obstacle_vertices = get_vertice_rect(obs)
                is_collide = separating_axis_theorem(car_vertices, obstacle_vertices)

                if is_collide:
                    obstacle_colliding.append(obs)
                    break

        # 모드에 따라 reroute할것인지 급정거 할 것인지 설정
        if len(obstacle_colliding) != 0:
            if self.mode == 'static':
                cx, cy, cyaw = self.collision_reroute(cx, cy, cyaw, obstacle_colliding)
            if self.mode == 'dynamic':
                self.estop = True

        return cx, cy, cyaw

    def collision_reroute(self, cx, cy, cyaw, obstacle_colliding):
        step = 45
        # step_region = 15
        obs_first = obstacle_colliding[-1]
        obs_end = obstacle_colliding[0]

        # dx_f = [obs_first[0] - icx for icx in cx] # Find the x-axis of the front axle relative to the path
        # dy_f = [obs_first[1] - icy for icy in cy] # Find the y-axis of the front axle relative to the path

        dx_e = [obs_end[0] - icx for icx in cx] # Find the x-axis of the front axle relative to the path
        dy_e = [obs_end[1] - icy for icy in cy] # Find the y-axis of the front axle relative to the path

        # d_f = np.hypot(dx_f, dy_f) # Find the distance from the front axle to the path
        # target_idx_f = np.argmin(d_f)

        d_e = np.hypot(dx_e, dy_e) # Find the distance from the front axle to the path
        target_idx_e = np.argmin(d_e)

        self.get_logger().info('obstacle detected!!  target idx : %d' %(target_idx_e))


        # if target_idx_e + (step+10) >= len(cyaw) or target_idx_f - (step+10) <= 0: # 10 대신 step_region
            # return cx, cy, cyaw
        if target_idx_e + (step+5) >= len(cyaw) or target_idx_e - (step+5) <= 0: # 10 대신 step_region
            return cx, cy, cyaw

        # Points to leave path
        start_x= cx[target_idx_e - step]
        start_y= cy[target_idx_e - step]
        start_yaw= cyaw[target_idx_e - step]
        start = (start_x, start_y, np.rad2deg(start_yaw))

        end_x = cx[target_idx_e + step]
        end_y = cy[target_idx_e + step]
        end_yaw = cyaw[target_idx_e + step]
        end = (end_x, end_y,  np.rad2deg(end_yaw))

        region1_x = cx[target_idx_e] - 10
        region1_y = cy[target_idx_e] - 10
        region2_x = cx[target_idx_e] + 10
        region2_y = cy[target_idx_e] + 10

        hy_a_star = hybrid_a_star(region1_x, region2_x,
                                  region1_y, region2_y,
                                  obstacle = self.obstacles,
                                  resolution = 0.5,
                                  length = self.L, width = self.W)
        reroute_path = hy_a_star.find_path(start, end)

        if reroute_path is None:
          self.get_logger().info("시간 초과, 회피경로 재탐색")
          self.is_fail = True
          return cx,cy,cyaw

        # resolution은 항상 self.W 보다 작아야함.

        rcx_ = []
        rcy_ = []
        rcyaw_ = []
        for i in range(len(reroute_path)):
            rcx_.append(reroute_path[i][0])
            rcy_.append(reroute_path[i][1])
            rcyaw_.append(reroute_path[i][2])
        # rcx_.append(cx[(target_idx_e + step + 1)])
        # rcy_.append(cy[(target_idx_e + step + 1)])

        cx_ = CubicSpline(range(len(rcx_)), rcx_)
        cy_ = CubicSpline(range(len(rcy_)), rcy_)
        dx = cx_(np.arange(0, len(rcx_) - 1, 0.2))
        dy = cy_(np.arange(0, len(rcy_) - 1, 0.2))
        rcyaw = np.arctan2(dy[1:] - dy[:-1], dx[1:] - dx[:-1])
        rcx = dx[:-1]
        rcy = dy[:-1]

        # stiching to form new path
        cx   = np.concatenate(( cx[0 : target_idx_e - step+1], rcx, cx[(target_idx_e + step+1) : ]))
        cy   = np.concatenate(( cy[0 : target_idx_e - step+1], rcy, cy[(target_idx_e + step+1) : ]))
        cyaw   = np.concatenate(( cyaw[0 : target_idx_e - step+1], rcyaw, cyaw[(target_idx_e + step+1) : ]))

        # print('Generated dev path')
        return cx, cy, cyaw


    def find_path(self):
        self.is_fail = False
        if len(self.ax) < 2:
            return
        cx_ = CubicSpline(range(len(self.ax)), self.ax)
        cy_ = CubicSpline(range(len(self.ay)), self.ay)
        dx = cx_(np.arange(0, len(self.ax) - 1, 0.1))
        dy = cy_(np.arange(0, len(self.ay) - 1, 0.1))
        cyaw = np.arctan2(dy[1:] - dy[:-1], dx[1:] - dx[:-1])
        cx = dx[:-1]
        cy = dy[:-1]

        if self.mode == 'dynamic' or self.mode == 'static':
            cx, cy, cyaw = self.determine_path(cx, cy, cyaw)
        if self.is_fail == True:
            return

        estop = Bool()
        estop.data = self.estop
        self.estop_pub.publish(estop)

        path_length = min(len(cx), len(cy), len(cyaw))

        self.target_path = Path2D()
        self.viz_path = Path()

        self.viz_path.header.frame_id = "map"
        self.viz_path.header.stamp = self.get_clock().now().to_msg()

        for n in range(0, path_length):
            # Appending to Target Path
            npose = Pose2D()
            npose.x = cx[n]
            npose.y = cy[n]
            npose.theta = cyaw[n]
            self.target_path.poses.append(npose)

            # Appending to Visualization Path
            vpose = PoseStamped()
            vpose.header.frame_id = "map"
            vpose.header.stamp = self.get_clock().now().to_msg()
            vpose.pose.position.x = cx[n]
            vpose.pose.position.y = cy[n]
            vpose.pose.position.z = 0.0
            vpose.pose.orientation = yaw_to_quaternion(np.pi * 0.5 - cyaw[n])
            self.viz_path.poses.append(vpose)

        self.local_planner_pub.publish(self.target_path)
        self.path_viz_pub.publish(self.viz_path)

def main(args=None):
    rclpy.init(args=args)

    try:
        local_planner = LocalPathPlanner()
        rclpy.spin(local_planner)

    finally:
        local_planner.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
