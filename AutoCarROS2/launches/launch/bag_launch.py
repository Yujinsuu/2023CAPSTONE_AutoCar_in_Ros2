import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    odom = 'autocar_odom'

    rviz = os.path.join(get_package_share_directory(odom), 'rviz', 'view.rviz')


    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'
        ),

        SetEnvironmentVariable(
            'RCUTILS_COLORIZED_OUTPUT', '1'
        ),

        Node(
            package = odom,
            name = 'tf_pub',
            executable = 'tf_pub.py'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],
            output={'both': 'log'}
        )
    ])

def main():

    generate_launch_description()

if __name__ == '__main__':
    main()
