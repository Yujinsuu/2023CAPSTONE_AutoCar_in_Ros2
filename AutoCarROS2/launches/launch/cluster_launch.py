from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'car', 'velodyne'],
            name = 'static_transform_publisher'
        ),
        Node(
            package = 'adaptive_clustering',
            name = 'adaptive_clustering',
            executable = 'adaptive_clustering'
        ),
        Node(
            package = 'autocar_map',
            name = 'obstacle_pub',
            executable = 'obstacle_pub.py'
        ),
        Node(
            package = 'autocar_map',
            name = 'obs_viz',
            executable = 'obstacle_viz.py'
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
