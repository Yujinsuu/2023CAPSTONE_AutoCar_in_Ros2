from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'car', 'velodyne'],
            name='static_transform_publisher'
        ),
        Node(
            package='adaptive_clustering',
            executable='adaptive_clustering',
            name='adaptive_clustering'
        ),
        Node(
            package='autocar_map',
            executable='obstacle_pub.py',
            name='obstacle_pub'
        ),
        Node(
            package='autocar_map',
            executable='obstacle_viz.py',
            name='obs_viz'
        ),
        Node(
            package='autocar_map',
            executable='wall_following.py',
            name='wall_follower'
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
