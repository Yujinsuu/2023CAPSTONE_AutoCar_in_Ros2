#!/usr/bin/env python3

import time
import numpy as np
from collections import deque, Counter

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Int32MultiArray, Float64MultiArray, Float64, Float32, String
from autocar_msgs.msg import LinkArray, State2D, Obstacle, VisionSteer
from ackermann_msgs.msg import AckermannDriveStamped


class Core(Node):

    def __init__(self):

        super().__init__('Core')

        # Initialise publishers
        self.autocar_pub = self.create_publisher(AckermannDriveStamped, '/autocar/autocar_cmd', 10)
        self.mission_status_pub = self.create_publisher(String, '/autocar/mission_status', 10)
        self.sign_angle_pub = self.create_publisher(Float32, '/sign_angle', 10)

        # Initialise subscribers
        self.ackermann_sub = self.create_subscription(AckermannDriveStamped, '/autocar/ackermann_cmd', self.command_cb, 10, callback_group=ReentrantCallbackGroup())
        self.links_sub = self.create_subscription(LinkArray, '/autocar/mode', self.links_cb, 10)
        self.state_sub = self.create_subscription(State2D, '/autocar/state2D', self.state_cb, 10)
        self.cte_sub = self.create_subscription(Float64, '/autocar/cte_error', self.cte_cb, 10)
        self.vision_sub = self.create_subscription(Float64MultiArray, '/lanenet_steer', self.vision_cb, 10)
        self.track_sub = self.create_subscription(VisionSteer, '/autocar/track_steer', self.track_cb, 10)
        self.obstacle_sub = self.create_subscription(Obstacle, '/autocar/obs_recog', self.obstacle_cb, 10)
        # self.tunnel_sub = self.create_subscription(String, '/tunnel_check', self.tunnel_check, 10)
        self.traffic_sub = self.create_subscription(String, '/traffic_sign', self.traffic_cb, 10)
        self.delivery_sub = self.create_subscription(Int32MultiArray, '/delivery_sign', self.delivery_cb, 10)
        self.delivery_stop_sub = self.create_subscription(Float32, '/delivery_stop', self.delivery_stop_cb, 10)

        # Class variables to use whenever within the class when necessary
        self.brake_stop = False
        self.t = 0
        self.dt = 0.1
        self.brake = 0.0

        self.link_num = 0
        self.waypoint = 0
        self.mode = 'global'
        self.traffic_stop_wp = 1e3
        self.parking_stop_wp = 1e3
        self.direction = 'None'
        self.next_path = 'straight'
        self.status = 'driving'
        self.time = 0.0

        self.target_speed = {'global': 25/3.6,  'curve': 10/3.6, 'parking': 6/3.6,     'rush':  7/3.6,    'revpark': 6/3.6,      'uturn':   1.5,
                             'static':  6/3.6, 'dynamic': 7/3.6,  'tunnel': 9/3.6, 'tollgate': 6/3.6, 'delivery_A': 4/3.6, 'delivery_B': 4/3.6,
                             'finish': 10/3.6}

        self.vel = 1.0
        self.cmd_speed = self.target_speed[self.mode]
        self.cmd_steer = 0.0
        self.gear = 0.0
        self.cte_term = 0.0

        self.obstacle_detected = 0
        self.obstacle = 'None'
        self.obs_distance = float(1e3)
        self.lane_detected = False
        self.vision_steer = 0.0
        self.cone_check = False
        self.track_steer = 0.0
        self.avoid_count = 0
        self.mission_count = 0
        self.tunnel_state = 'entry'

        self.yolo_light = ['None']
        self.traffic_stop = False
        self.pause = 0.0

        self.A_check = False
        self.A_num = 0
        self.sign_pose = 0
        self.distance = -1.0
        self.stop_wp = 1e3

        self.Mount_angle = 30
        self.Camera_angle = 78
        self.Image_size = 640

        queue_size = 35
        init_queue = [0 for _ in range(queue_size)]
        self.link_change = deque(init_queue, maxlen = queue_size)
        init_queue = ['global' for _ in range(queue_size)]
        self.mode_change = deque(init_queue, maxlen = queue_size)


    def state_cb(self,msg):
        self.vel = np.sqrt((msg.twist.x**2.0) + (msg.twist.y**2.0))
        if self.vel <= 1: self.vel = 1.0


    def command_cb(self, msg):
        self.cmd_speed = self.target_speed[self.mode]

        self.cmd_steer = msg.drive.steering_angle
        # if self.link_num == 0 and self.waypoint > 200:
        #     self.cmd_steer = self.vision_steer
        # elif self.link_num == 1 or 2:
        #     self.cmd_steer = self.vision_steer
        # elif self.link_num == 3 and self.status == 'complete':
        #     self.cmd_steer = self.vision_steer

        self.autocar_control()


    def cte_cb(self, msg):
        self.cte_term = msg.data

    def links_cb(self, msg):
        self.link_num = msg.link_num
        self.waypoint = msg.closest_wp
        self.mode = msg.mode
        self.traffic_stop_wp = msg.traffic_stop_wp
        self.parking_stop_wp = msg.parking_stop_wp
        self.direction = msg.direction
        self.next_path = msg.next_path

        # link change check
        self.link_change.append(self.link_num)
        self.mode_change.append(self.link_num)

    def obstacle_cb(self, msg):
        self.obstacle_detected = msg.detected
        self.obstacle = msg.obstacle
        self.obs_distance = msg.distance

    def vision_cb(self, msg):
        self.lane_detected = bool(msg.data[0])
        self.vision_steer = msg.data[1]

    def track_cb(self, msg):
        self.cone_check = msg.detected
        self.track_steer = msg.steer

    def tunnel_check(self, msg):
        self.tunnel_state = msg.data

    def traffic_cb(self, msg):
        self.yolo_light = msg.data.split(",")

    def delivery_stop_cb(self, msg):
        self.distance = msg.data

    def delivery_cb(self, msg):
        angle = Float32()

        if self.mode in ['delivery_A', 'delivery_B']:
            # msg = [A_id, A1x, A2x, A3x, B1x, B2x, B3x]

            if not self.A_check and msg.data[0] != -1:
                self.A_num = msg.data[0]
                self.A_check = True

            if self.A_check:
                if self.mode == 'delivery_A':
                    self.sign_pose = msg.data[self.A_num + 1]

                elif self.mode == 'delivery_B':
                    self.sign_pose = msg.data[self.A_num + 4]

            if not self.A_check and self.mode == 'delivery_B':
                self.sign_pose = msg.data[4]

            if self.sign_pose <= 100:
                angle.data = 0.0
            else:
                pixel_angle = self.Mount_angle + (self.sign_pose - self.Image_size/2) * (self.Camera_angle / self.Image_size)
                angle.data = np.deg2rad(-pixel_angle)

        else:
            self.sign_pose = 0
            angle.data = 0.0

        self.sign_angle_pub.publish(angle)


    def identify_traffic_light(self, path, wp):
        if path == 'straight': tf_light = ['Green', 'Straightleft']
        elif path == 'left': tf_light = ['Left', 'Straightleft']
        elif path == 'right':
            self.pause += self.dt
            if self.pause < 4:
                tf_light = []
            else:
                tf_light = ['Green', 'Left', 'Red', 'Straightleft', 'Yellow', 'None']
        else: tf_light = ['Green', 'Left', 'Red', 'Straightleft', 'Yellow', 'None']


        if len([i for i in self.yolo_light if i in tf_light]) == 0:
            self.traffic_stop = True

        else:
            self.traffic_stop = False
            self.t = 0


    def brake_control(self, b, m, t=3):
        self.brake = b * self.t
        self.t += self.dt
        if self.brake >= m: self.brake = m

        if self.t >= t: self.brake = 0
        elif self.brake_stop:
            self.cmd_speed = 0.0
            self.cmd_steer = 0.0


    def autocar_control(self):

        counter = Counter(self.link_change)
        value, count = counter.most_common(1)[0]

        if value != self.link_num:
            self.status = 'driving'
            self.traffic_stop = False
            self.pause = 0.0
            self.t = 0

            counter = Counter(self.mode_change)
            value, count = counter.most_common(1)[0]
            if value == 'global':
                self.brake = 150 * (self.target_speed['global'] - self.target_speed[self.mode]) / self.target_speed['global']
                if self.brake < 0: self.brake = 0

        else: self.brake = 0

        if self.mode == 'global':
            self.status = 'driving'

            if self.direction == 'Curve' or abs(np.rad2deg(self.cmd_steer)) >= 15 or abs(self.cte_term) >= 10:
                self.cmd_speed = self.target_speed['curve']
                self.brake = 30.0
            else:
                self.brake = 0.0

            if self.traffic_stop_wp <= 25:
                self.cmd_speed = self.target_speed['curve']
                if self.traffic_stop_wp <= 5:
                    self.identify_traffic_light(self.next_path, self.traffic_stop_wp)

                elif self.traffic_stop_wp > 15:
                    self.brake = 100 * (self.target_speed['global'] - self.target_speed['curve']) / self.target_speed['global']

                else:
                    self.brake = 0.0


        # elif self.mode == 'tollgate':
        #     if self.waypoint >= 10:
        #         if self.lane_detected:
        #             self.cmd_steer = self.vision_steer

        elif self.mode == 'uturn':
            if self.status == 'driving':
                if self.traffic_stop_wp <= 0:
                    self.status = 'complete'

                if self.waypoint >= 30 and self.obs_distance < 7:
                    self.brake = 0.0
                    self.t = 0
                    self.status = 'track'


            elif self.status == 'track':
                if self.waypoint > 216:
                    if abs(self.cte_term) <= 10:
                        self.avoid_count = time.time()
                        self.status = 'complete'

                self.cmd_steer = self.track_steer

            else:
                if time.time() - self.avoid_count > 3:
                    self.cmd_speed = self.target_speed['tollgate']


        elif self.mode == 'tunnel':
            if self.status == 'driving':
                # self.cmd_steer = self.vision_steer
                if self.waypoint >= 40:
                    self.status = 'lanenet'

            elif self.status == 'lanenet':
                # self.cmd_steer = self.vision_steer

                if self.obstacle == 'dynamic':
                    self.mission_count += 1
                    self.avoid_count = time.time()
                    self.status = 'stop'
                    self.brake = 0.0
                    self.t = 0

                if self.obstacle == 'static':
                    self.mission_count += 1
                    self.avoid_count = time.time()
                    self.status = 'avoid'
                    self.brake = 0.0
                    self.t = 0

                if self.traffic_stop_wp <= 40:
                    self.status = 'complete'
                    self.brake = 0.0
                    self.t = 0

            elif self.status == 'stop':
                self.cmd_speed = 0.0
                self.cmd_steer = 0.0

                brake_force = 500
                max_brake = 200
                self.brake_control(brake_force, max_brake, 2)

                if self.obstacle == 'dynamic':
                    self.avoid_count = time.time()

                if time.time() - self.avoid_count >= 1.5:
                    self.status = 'lanenet'
                    self.t = 0

            elif self.status == 'avoid':
                self.cmd_speed = self.target_speed['static']

                brake_force = 30
                max_brake = 100
                self.brake_control(brake_force, max_brake, 2)

                if self.obstacle == 'static':
                    self.avoid_count = time.time()

                if time.time() - self.avoid_count >= 2:
                    self.status = 'lanenet'
                    self.t = 0


        elif self.mode == 'static':
            if self.status == 'driving':
                self.cmd_speed = self.target_speed['curve']
                if self.traffic_stop_wp <= 15:
                    self.status = 'complete'

                elif self.obstacle_detected:
                    self.avoid_count = time.time()
                    self.status = 'avoid'
                    self.t = 0

            elif self.status == 'avoid':
                brake_force = 30
                max_brake = 100
                self.brake_control(brake_force, max_brake, 2)

                if self.obstacle_detected:
                    self.avoid_count = time.time()

                if time.time() - self.avoid_count >= 2:
                    self.status = 'driving'
                    self.t = 0

            else:
                self.cmd_speed = self.target_speed['curve']

                if self.traffic_stop_wp <= 5:
                    self.identify_traffic_light(self.next_path, self.traffic_stop_wp)


        elif self.mode == 'dynamic':
            if self.status == 'driving':
                if self.traffic_stop_wp <= 10:
                    self.status = 'complete'

                elif self.obstacle_detected:
                    self.avoid_count = time.time()
                    self.status = 'stop'

            elif self.status == 'stop':
                self.cmd_speed = 0.0
                self.cmd_steer = 0.0

                brake_force = 500
                max_brake = 200
                self.brake_control(brake_force, max_brake, 2)

                if self.obstacle_detected:
                    self.avoid_count = time.time()

                if time.time() - self.avoid_count >= 1.5:
                    self.status = 'driving'
                    self.t = 0

            else:
                self.cmd_speed = self.target_speed['curve']

                if self.traffic_stop_wp <= 5:
                    self.identify_traffic_light(self.next_path, self.traffic_stop_wp)


        elif self.mode == 'parking':
            if self.status == 'driving':
                self.t = 0
                self.status = 'parking'

            elif self.status == 'parking':
                if self.waypoint > 40:
                    self.status = 'complete'

                elif self.parking_stop_wp <= 9:
                    self.status = 'return'

            elif self.status == 'return':
                self.brake_stop = True
                self.gear = 2.0
                self.cmd_speed = self.target_speed['parking']

                if self.parking_stop_wp <= 10:
                    self.cmd_speed = self.target_speed['rush']

                    brake_force = 150
                    max_brake = 200
                    self.brake_control(brake_force, max_brake, 3)

                elif self.parking_stop_wp <= 12:
                    self.brake = 20.0
                    self.t = 0

                elif self.parking_stop_wp >= 16:
                    self.gear = 0.0

                    brake_force = 150
                    max_brake = 200
                    self.brake_control(brake_force, max_brake, 2)

                    if self.t >= 2:
                        self.brake_stop = False
                        self.status = 'complete'
                        self.t = 0

                else:
                    self.brake = 0.0

            else:
                self.cmd_speed = self.target_speed['global']

                if self.traffic_stop_wp <= 5:
                    self.identify_traffic_light(self.next_path, self.traffic_stop_wp)


        elif self.mode == 'revpark':
            if self.status == 'driving':
                self.t = 0
                if self.traffic_stop_wp <= 10:
                    self.status = 'parking'
                if self.waypoint >= 60:
                    self.status = 'complete'

            elif self.status == 'parking':
                self.brake_stop = True
                self.gear = 2.0

                if self.traffic_stop_wp <= 10:
                    self.cmd_speed = self.target_speed['rush']

                    brake_force = 150
                    max_brake = 200
                    self.brake_control(brake_force, max_brake, 2)

                elif self.traffic_stop_wp <= 12:
                    self.brake = 20.0

                elif self.parking_stop_wp <= 13:
                    self.t = 0
                    self.gear = 0.0
                    self.status = 'return'

                else:
                    self.brake = 0.0

            elif self.status == 'return':
                if self.parking_stop_wp <= 14:
                    self.cmd_speed = self.target_speed['rush']

                    brake_force = 150
                    max_brake = 200
                    self.brake_control(brake_force, max_brake, 3)

                elif self.parking_stop_wp <= 15:
                    self.brake_stop = False
                    self.brake = 20.0
                    self.t = 0

                elif self.parking_stop_wp >= 25:
                    self.status = 'complete'
                    self.brake_stop = False

                else:
                    self.brake = 0.0
                    self.t = 0

            else:
                self.cmd_speed = self.target_speed['global']

                if self.traffic_stop_wp <= 5:
                    self.identify_traffic_light(self.next_path, self.traffic_stop_wp)


        elif self.mode == 'delivery_A':
            if self.status == 'driving':
                if self.waypoint > 30:
                    self.status = 'check'

            elif self.status == 'check':
                if self.distance != -1:
                    self.stop_wp = self.waypoint + int(self.distance) -1
                    self.t = 0
                    self.status = 'detected'

                if self.sign_pose >= 500:
                    self.t = 0
                    self.status = 'stop'

                if self.traffic_stop_wp <= 15:
                    self.status = 'complete'

            elif self.status == 'detected':

                if self.stop_wp - self.waypoint <= 0:
                    self.status = 'stop'

            elif self.status == 'stop':
                self.cmd_speed = 0.0
                self.cmd_steer = 0.0

                brake_force = 200
                max_brake = 200
                self.brake_control(brake_force, max_brake, 2)

                if self.t >= 6:
                    self.status = 'complete'
                    self.t = 0

            else:
                self.cmd_speed = self.target_speed['global']

                if self.traffic_stop_wp <= 5:
                    self.identify_traffic_light(self.next_path, self.traffic_stop_wp)

        elif self.mode == 'delivery_B':
            if self.status == 'driving':
                if self.waypoint > 40:
                    self.status = 'check'

                elif self.waypoint < 15:
                    self.cmd_speed = self.target_speed['curve']

            elif self.status == 'check':
                if self.distance != -1:
                    self.stop_wp = self.waypoint + int(self.distance) -1
                    self.t = 0
                    self.status = 'detected'

                if self.sign_pose >= 500:
                    self.t = 0
                    self.status = 'stop'

                if self.traffic_stop_wp <= 50:
                    self.status = 'complete'

            elif self.status == 'detected':

                if self.stop_wp - self.waypoint <= 0:
                    self.status = 'stop'

            elif self.status == 'stop':
                self.cmd_speed = 0.0
                self.cmd_steer = 0.0

                brake_force = 200
                max_brake = 200
                self.brake_control(brake_force, max_brake, 2)

                if self.t >= 6:
                    self.status = 'complete'
                    self.t = 0

            else:
                self.cmd_speed = self.target_speed['curve']

                if self.traffic_stop_wp <= 5:
                    self.identify_traffic_light(self.next_path, self.traffic_stop_wp)

        elif self.mode == 'finish':
            if self.waypoint >= 50:
                self.status = 'complete'
                self.cmd_speed = 0.0
                self.cmd_steer = 0.0

                brake_force = 250
                max_brake = 200
                self.brake_control(brake_force, max_brake, 2)

        self.publish_autocar_command()


    def publish_autocar_command(self):
        status = String()
        status.data = self.status

        self.mission_status_pub.publish(status)

        car = AckermannDriveStamped()
        car.header.frame_id = 'odom'
        car.header.stamp = self.get_clock().now().to_msg()

        car.drive.acceleration = self.gear
        car.drive.jerk = float(self.brake)

        if self.traffic_stop:
            car.drive.jerk = 200.0
            car.drive.steering_angle = 0.0
            car.drive.speed = 0.0
        else:
            car.drive.steering_angle = self.cmd_steer
            self.time+=self.dt
            car.drive.speed = self.cmd_speed

        self.autocar_pub.publish(car)


def main(args=None):

    # Initialise the node
    rclpy.init(args=args)

    try:
        # Initialise the class
        core = Core()

        # Stop the node from exiting
        rclpy.spin(core)

    finally:
        core.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
