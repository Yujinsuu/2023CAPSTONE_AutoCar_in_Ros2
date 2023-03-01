#!/usr/bin/env python3
# -*- coding: utf8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Int32, Float32
from ackermann_msgs.msg import AckermannDriveStamped
#from geometry_msgs.msg import Twist

from math import *
import numpy as np
import serial
import math
import time

fast_flag =False
S = 0x53
T = 0x54
X = 0x58
AorM = 0x01
ESTOP = 0x00
GEAR = 0x00
SPEED0 = 0x00
SPEED1 = 0x00
STEER0 = 0X02
STEER1 = 0x02
BRAKE = 0x01
ALIVE = 0
ETX0 = 0x0d
ETX1 = 0x0a
Packet=[]
read=[]
count=0
gear = 0
speed = 0
steer = 0
brake = 0
count_alive=0
cur_ENC_backup=0

class erp42(Node):

	def __init__(self):
		super().__init__('erp42')
		self.ackermann_subscriber = self.create_subscription(AckermannDriveStamped, '/autocar/ackermann_cmd',
		self.acker_callback, 10)
		self.ser = serial.serial_for_url("/dev/ttyUSB0", baudrate=115200, timeout=1)
		self.timer = self.create_timer(0.3, self.timer_callback)

	def GetAorM(self):
		AorM = 0x01
		return  AorM

	def GetESTOP(self):
		ESTOP = 0x00
		return  ESTOP

	def GetGEAR(self, gear):
		GEAR = gear
		return  GEAR

	def GetSPEED(self, speed):
		global count
		SPEED0 = 0x00
		SPEED = int(speed*36) # float to integer
		SPEED1 = abs(SPEED) # m/s to km/h*10
		return SPEED0, SPEED1

	def GetSTEER(self, steer): # steer은 rad/s 값으로 넣어줘야한다.
		steer=steer*71*(180/pi) # rad/s to degree/s*71

		if(steer>=2000):
			steer=1999
		elif(steer<=-2000):
			steer=-1999
		steer_max=0b0000011111010000 # +2000
		steer_0 = 0b0000000000000000
		steer_min=0b1111100000110000 # -2000

		if (steer>=0):
			angle=int(steer)
			STEER=steer_0+angle
		else:
			angle=int(-steer)
			angle=2000-angle
			STEER=steer_min+angle

		STEER0=STEER & 0b1111111100000000
		STEER0=STEER0 >> 8
		STEER1=STEER & 0b0000000011111111
		return STEER0, STEER1

	def GetBRAKE(self, brake):
		BRAKE = brake
		return  BRAKE

	def Send_to_ERP42(self, gear, speed, steer, brake):
		global S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1, count_alive
		count_alive = count_alive+1

		if count_alive==0xff:
			count_alive=0x00

		AorM = self.GetAorM()
		ESTOP = self.GetESTOP()
		GEAR = self.GetGEAR(gear)
		SPEED0, SPEED1 = self.GetSPEED(speed)
		STEER0, STEER1 = self.GetSTEER(steer)
		BRAKE = self.GetBRAKE(brake)

		ALIVE = count_alive

		vals = [S, T, X, AorM, ESTOP,GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1]
		# print(vals[8], vals[9])
		# print(hex(vals[8]), hex(vals[9]))
		# print(vals[8].to_bytes(1, byteorder='big'),vals[9].to_bytes(1, byteorder='big'))
		for i in range(len(vals)):
			self.ser.write(vals[i].to_bytes(1, byteorder='big')) # send!

		# for i in range(8, 10):
		# 	self.ser.write(vals[i].to_bytes(1, byteorder='big')) # send!

	def acker_callback(self, msg):
		global speed, steer, brake, gear
		speed = msg.drive.speed
		steer = -(msg.drive.steering_angle)  # why minus?
		brake = int(msg.drive.jerk)
		gear = int(msg.drive.acceleration)

	def timer_callback(self):
		# steer=radians(float(input("steer_angle:")))
		print("speed:",speed, "steer:", steer*180/np.pi, "brake",brake, "gear", gear)
		self.Send_to_ERP42(gear, speed, steer, brake)

def main(args=None):
	rclpy.init(args=args)
	node = erp42()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
