#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointCloud, ChannelFloat32
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point32
from autocar_msgs.msg import State2D

class LidarDataProcessor(Node):

    def __init__(self):
        super().__init__('lidar_data_processor')
        self.pointcloud = PointCloud()
        self.pointcloud.header.stamp = self.get_clock().now().to_msg()
        self.pointcloud.header.frame_id = 'velodyne'
        self.channel = ChannelFloat32()
        self.channel.name = 'intensity'
        self.pointcloud.channels.append(self.channel)
        self.subscription = self.create_subscription( PointCloud2, '/velodyne_points', self.lidar_callback, 10)
        self.subscription = self.create_subscription(State2D, '/autocar/state2D',self.state2D_cb , 10)
        self.publisher = self.create_publisher( PointCloud, '/upper_points', 10)
        
        self.px = 0.0
        self.py = 0.0
       
    def state2D_cb(self, msg):

        self.px = msg.pose.x
        self.py = msg.pose.y


    def lidar_callback(self, msg):
        # Read the point cloud data
        pc_data = list(point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=True))
        filtered_list = [tup for tup in pc_data if tup[-1] == 15]
        xyz_data = [(point[0], point[1], point[2],  point[3]) for point in filtered_list]
        #print('filtered_list',filtered_list)
        intensity = []
        self.pointcloud.points.clear()
        min_z = 100.0
        for x, y, z, int in xyz_data:
            point = Point32()
            point.x = x
            point.y = y
            point.z = z
            if z > 2 and x < 5 and x > 0:
                intensity.append(int)   
                self.pointcloud.points.append(point)
                print('z', z)
                if z < min_z:
                    min_z = z


        if len(self.pointcloud.points) == 0:
            self.outside_of_tunnel = True
        else:
            self.outside_of_tunnel = False
            
        if self.outside_of_tunnel:
            print('px: ',self.px, 'py: ', self.py)
        
        print(len(self.pointcloud.points))
        print('mode',self.outside_of_tunnel)
            
        
 
        self.channel.values = intensity

        self.publisher.publish(self.pointcloud)



def main(args=None):

    # Initialise the node
    rclpy.init(args=args)

    try:
        # Initialise the class
        lidar_data_processor = LidarDataProcessor()

        # Stop the node from exiting
        rclpy.spin(lidar_data_processor)

    finally:
        lidar_data_processor.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()