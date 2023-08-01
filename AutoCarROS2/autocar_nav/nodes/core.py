#!/usr/bin/env python3

import time
import numpy as np
from collections import deque, Counter

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Float64MultiArray, Float32, String, Bool
from autocar_msgs.msg import LinkArray, State2D
from ackermann_msgs.msg import AckermannDriveStamped


class Core(Node):

    def __init__(self):

        super().__init__('Core')

        # Initialise publishers
        self.autocar_pub = self.create_publisher(AckermannDriveStamped, '/autocar/autocar_cmd', 10)
        self.mission_status_pub = self.create_publisher(String, '/autocar/mission_status', 10)
        self.sign_angle_pub = self.create_publisher(Float32, '/sign_angle', 10)

        # Initialise subscribers
        self.ackermann_sub = self.create_subscription(AckermannDriveStamped, '/autocar/ackermann_cmd', self.command_cb, 10)
        self.links_sub = self.create_subscription(LinkArray, '/autocar/mode', self.links_cb, 10)
        self.state_sub = self.create_subscription(State2D, '/autocar/state2D', self.state_cb, 10)
        self.vision_sub = self.create_subscription(Float64MultiArray, '/lanenet_steer', self.vision_cb, 10)
        self.obstacle_sub = self.create_subscription(Bool, '/autocar/estop', self.obstacle_cb, 10)
        self.inf_sub = self.create_subscription(Int32MultiArray, '/tunnel_check', self.tunnel_check, 10)
        self.traffic_sub = self.create_subscription(String, '/traffic_sign', self.traffic_cb, 10)
        self.delivery_sub = self.create_subscription(Int32MultiArray, '/delivery_sign', self.delivery_cb, 10)
        self.delivery_stop_sub = self.create_subscription(Float32, '/delivery_stop', self.delivery_stop_cb, 10)

        # Class variables to use whenever within the class when necessary
        self.brake_stop = False
        self.t = 0
        self.dt = 0.1
        self.brake = 0.0

        self.waypoint = 0
        self.mode = 'global'
        self.traffic_stop_wp = 1e3
        self.parking_stop_wp = 1e3
        self.direction = 'Straight'
        self.next_path = 'straight'
        self.status = 'driving'

        self.target_speed = {'global': 18/3.6,   'curve': 8/3.6, 'parking': 4/3.6,     'rush': 6.5/3.6,    'revpark': 4/3.6,      'uturn': 5/3.6,
                             'static':  5/3.6, 'dynamic': 5/3.6,  'tunnel': 7/3.6, 'tollgate':   6/3.6, 'delivery_A': 4/3.6, 'delivery_B': 4/3.6,
                             'finish': 10/3.6}

        self.vel = 1.0
        self.cmd_speed = self.target_speed[self.mode]
        self.cmd_steer = 0.0
        self.gear = 0.0

        self.dynamic_obstacle = False
        self.lane_detected = False
        self.vision_steer = 0.0
        self.avoid_count = 0
        self.tunnel_state = 'outside'

        self.yolo_light = 'Straightleft'
        self.traffic_stop = False
        self.pause = 0.0

        self.A_check = False
        self.A_num = 0
        self.sign_pose = 0
        self.distance = 0.0
        self.stop_wp = 1e3

        self.Mount_angle = 30
        self.Camera_angle = 78
        self.Image_size = 640

        queue_size = 29
        init_queue = ['global' for _ in range(queue_size)]
        self.queue = deque(init_queue, maxlen = queue_size)


    def state_cb(self,msg):
        self.vel = np.sqrt((msg.twist.x**2.0) + (msg.twist.y**2.0))
        if self.vel <= 1: self.vel = 1.0


    def command_cb(self, msg):
        self.cmd_speed = self.target_speed[self.mode]

        self.cmd_steer = msg.drive.steering_angle

        self.autocar_control()


    def links_cb(self, msg):
        self.waypoint = msg.closest_wp
        self.mode = msg.mode
        self.traffic_stop_wp = msg.traffic_stop_wp
        self.parking_stop_wp = msg.parking_stop_wp
        self.direction = msg.direction
        self.next_path = msg.next_path

        # mode change check
        self.queue.append(self.mode)

    def obstacle_cb(self, msg):
        self.dynamic_obstacle = msg.data

    def vision_cb(self, msg):
        self.lane_detected = bool(msg.data[0])
        self.vision_steer = msg.data[1]

    def tunnel_check(self, msg):
        scan_range = msg.data[-1]
        inf_index = np.array(msg.data)

        f_inf = len(inf_index[inf_index > 0.6 * scan_range])
        r_inf = len(inf_index[inf_index < 0.4 * scan_range])
        threshold = 0.2 * scan_range

        if self.mode == 'tunnel' and self.status == 'lanenet':
            if self.tunnel_state == 'outside':
                if (f_inf <= threshold <= r_inf) or (f_inf < threshold and r_inf < threshold):
                    self.tunnel_state = 'inside'
            elif self.tunnel_state == 'inside':
                if (f_inf >= threshold >= r_inf) or (f_inf > threshold and r_inf > threshold):
                    self.tunnel_state = 'outside'


    def traffic_cb(self, msg):
        self.yolo_light = msg.data

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
        if path != 'right': self.pause = 0.0

        if path == 'straight': tf_light = ['Green', 'Straightleft']
        elif path == 'left': tf_light = ['Left', 'Straightleft']
        elif path == 'right':
            self.pause += self.dt
            if self.pause < 2:
                self.traffic_stop = True
                tf_light = []
            else:
                self.traffic_stop = False
                tf_light = ['Green', 'Left', 'Red', 'Straightleft', 'Yellow', 'None']
        else: tf_light = ['Green', 'Left', 'Red', 'Straightleft', 'Yellow', 'None']

        if self.yolo_light not in tf_light:
            self.traffic_stop = True
            brake_force = (4 - wp) * 50
            max_brake = 100
            self.brake_control(brake_force, max_brake, 3)

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

        counter = Counter(self.queue)
        value, count = counter.most_common(1)[0]

        if self.mode != 'global':
            if value != self.mode:
                self.status = 'driving'
                self.brake = 100 * (self.target_speed['global'] - self.target_speed[self.mode]) / self.target_speed['global']
                if self.brake < 0: self.brake = 0
            else: self.brake = 0

        if self.mode == 'global':
            self.status = 'driving'

            if self.direction == 'Curve' or abs(np.rad2deg(self.cmd_steer)) >= 10:
                self.cmd_speed = self.target_speed['curve']
                self.brake = 30.0
            else:
                self.brake = 0.0

            if self.traffic_stop_wp <= 3:
                self.identify_traffic_light(self.next_path, self.traffic_stop_wp)
            else: self.t = 0


        elif self.mode == 'tunnel':
            if self.status == 'driving':
                self.status = 'lanenet'

            elif self.status == 'lanenet':
                if self.tunnel_state == 'outside' or self.lane_detected:
                    self.cmd_steer = self.vision_steer

                if self.dynamic_obstacle:
                    self.avoid_count = time.time()
                    self.status = 'avoid'

                if self.traffic_stop_wp <= 0:
                    self.status = 'complete'
                    self.brake = 0.0
                    self.t = 0

            elif self.status == 'avoid':
                self.cmd_speed = self.target_speed['static']

                brake_force = 30
                max_brake = 100
                self.brake_control(brake_force, max_brake, 2)

                if self.dynamic_obstacle:
                    self.avoid_count = time.time()

                if self.t >= 5 and time.time() - self.avoid_count >= 2:
                    self.status = 'lanenet'
                    self.t = 0


        elif self.mode == 'dynamic':
            if self.status == 'driving':
                if self.dynamic_obstacle:
                    self.avoid_count = time.time()
                    self.status = 'stop'

            elif self.status == 'stop':
                self.cmd_speed = 0.0
                self.cmd_steer = 0.0

                brake_force = 150
                max_brake = 100
                self.brake_control(brake_force, max_brake, 2)

                if self.dynamic_obstacle:
                    self.avoid_count = time.time()

                if time.time() - self.avoid_count >= 2:
                    self.status = 'complete'
                    self.t = 0

            else:
                self.cmd_speed = self.target_speed['global']

                if self.traffic_stop_wp <= 3:
                    self.identify_traffic_light(self.next_path, self.traffic_stop_wp)


        elif self.mode == 'parking':
            if self.status == 'driving':
                self.t = 0
                self.status = 'parking'

            elif self.status == 'parking':
                if self.traffic_stop_wp <= 10:
                    self.status = 'complete'

                elif self.parking_stop_wp <= 2:
                    self.status = 'return'

            elif self.status == 'return':
                self.brake_stop = True
                self.gear = 2.0
                self.cmd_speed = self.target_speed['parking']

                if self.parking_stop_wp <= 3:
                    self.cmd_speed = self.target_speed['rush']

                    brake_force = 150
                    max_brake = 100
                    self.brake_control(brake_force, max_brake, 3)

                elif self.parking_stop_wp <= 5:
                    self.brake = 20.0
                    self.t = 0

                elif self.parking_stop_wp >= 12:
                    self.gear = 0.0

                    brake_force = 150
                    max_brake = 100
                    self.brake_control(brake_force, max_brake, 2)

                    if self.t >= 2:
                        self.brake_stop = False
                        self.status = 'complete'
                        self.t = 0

                else:
                    self.brake = 0.0

            else:
                self.cmd_speed = self.target_speed['global']

                if self.traffic_stop_wp <= 3:
                    self.identify_traffic_light(self.next_path, self.traffic_stop_wp)

        elif self.mode == 'revpark':
            if self.status == 'driving':
                self.t = 0
                if self.traffic_stop_wp <= 15:
                    self.status = 'parking'

            elif self.status == 'parking':
                self.brake_stop = True
                self.gear = 2.0

                if self.traffic_stop_wp <= 16:
                    self.cmd_speed = self.target_speed['rush']

                    brake_force = 150
                    max_brake = 100
                    self.brake_control(brake_force, max_brake, 2)

                elif self.traffic_stop_wp <= 18:
                    self.brake = 20.0

                elif self.parking_stop_wp <= 2:
                    self.t = 0
                    self.gear = 0.0
                    self.status = 'return'

                else:
                    self.brake = 0.0

            elif self.status == 'return':
                if self.parking_stop_wp <= 3:
                    self.cmd_speed = self.target_speed['rush']

                    brake_force = 150
                    max_brake = 100
                    self.brake_control(brake_force, max_brake, 3)

                    if self.t >= 3:
                        self.brake_stop = False

                elif self.parking_stop_wp <= 5:
                    self.brake = 20.0

                elif self.parking_stop_wp >= 15:
                    self.status = 'complete'
                    self.brake_stop = False

                else:
                    self.brake = 0.0
                    self.t = 0

            else:
                self.cmd_speed = self.target_speed['global']

                if self.traffic_stop_wp <= 3:
                    self.identify_traffic_light(self.next_path, self.traffic_stop_wp)


        elif self.mode in ['delivery_A', 'delivery_B']:
            if self.status == 'driving':
                if self.distance != -1:
                    self.stop_wp = self.waypoint + int(self.distance)
                    self.t = 0
                    self.status = 'check'

                if self.sign_pose >= 415:
                    self.t = 0
                    self.status = 'stop'

            elif self.status == 'check':
                if self.stop_wp - self.waypoint <= 0:
                    self.status = 'stop'

            elif self.status == 'stop':
                self.cmd_speed = 0.0
                self.cmd_steer = 0.0

                brake_force = 150
                max_brake = 100
                self.brake_control(brake_force, max_brake, 2)

                if self.t > 2:
                    self.status = 'complete'
                    self.t = 0

            else:
                self.cmd_speed = self.target_speed['global']

                if self.traffic_stop_wp <= 3:
                    self.identify_traffic_light(self.next_path, self.traffic_stop_wp)

        elif self.mode == 'finish':
            if self.waypoint >= 15:
                self.cmd_speed = 0.0
                self.cmd_steer = 0.0

                brake_force = 150
                max_brake = 100
                self.brake_control(brake_force, max_brake, 2)


        self.publish_autocar_command()


    def publish_autocar_command(self):
        status = String()
        status.data = self.status

        self.mission_status_pub.publish(status)

        car = AckermannDriveStamped()
        car.header.frame_id = 'odom'
        car.header.stamp = self.get_clock().now().to_msg()

        if self.traffic_stop:
            car.drive.steering_angle = 0.0
            car.drive.speed = 0.0
        else:
            car.drive.steering_angle = self.cmd_steer
            car.drive.speed = self.cmd_speed

        car.drive.acceleration = self.gear
        car.drive.jerk = float(self.brake)

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
