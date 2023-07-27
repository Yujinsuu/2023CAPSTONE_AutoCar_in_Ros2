import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

def generate_launch_description():

    cam = 'v4l2_camera'

    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'
        ),
        SetEnvironmentVariable(
            'RCUTILS_COLORIZED_OUTPUT', '1'
        ),

        DeclareLaunchArgument(
            'front', default_value='/dev/video4'
        ),
        DeclareLaunchArgument(
            'side', default_value='/dev/video2'
        ),
        DeclareLaunchArgument(
            'lane', default_value='/dev/video0'
        ),

        Node(
            package=cam,
            name='v4l2_camera',
            namespace='/front',
            executable='v4l2_camera_node',
            parameters=[{'video_device': LaunchConfiguration('front')}],
        ),
        Node(
            package=cam,
            name='v4l2_camera',
            namespace='/side',
            executable='v4l2_camera_node',
            parameters=[{'video_device': LaunchConfiguration('side')}],
        ),
        Node(
            package=cam,
            name='v4l2_camera',
            namespace='/lane',
            executable='v4l2_camera_node',
            parameters=[{'video_device': LaunchConfiguration('lane')}],
        )
    ])

def main():

    generate_launch_description()

if __name__ == '__main__':
    main()
