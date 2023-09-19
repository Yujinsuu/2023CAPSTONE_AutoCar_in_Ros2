import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np

class LidarDataProcessor(Node):

    def __init__(self):
        super().__init__('lidar_data_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',  # 라이다 데이터를 발행하는 토픽 이름으로 변경해야 합니다.
            self.lidar_callback,
            10
        )

    def lidar_callback(self, msg):
        # Read the point cloud data
        pc_data = PointCloud2.read_points(msg, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=True)

        # Extract the ring channel data (assuming "ring" channel represents angles)
        angles = [point[4] for point in pc_data]

        # Find the index of the point with the highest angle
        max_angle_index = np.argmax(angles)

        # Extract the data point corresponding to the highest angle
        max_angle_data = pc_data[max_angle_index]

        # Now max_angle_data contains the data point corresponding to the highest angle
        x, y, z, intensity, ring = max_angle_data
        self.get_logger().info(f"Highest Angle Data: x={x}, y={y}, z={z}, intensity={intensity}, ring={ring}")

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