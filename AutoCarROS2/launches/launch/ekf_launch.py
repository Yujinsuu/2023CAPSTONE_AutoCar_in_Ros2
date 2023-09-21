import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    odom = 'autocar_odom'
    mappkg = 'autocar_map'

    rviz = os.path.join(get_package_share_directory(odom), 'rviz', 'view.rviz')
    ekf = os.path.join(get_package_share_directory(odom), 'config', 'ekf.yaml')


    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'
        ),

        SetEnvironmentVariable(
            'RCUTILS_COLORIZED_OUTPUT', '1'
        ),

        Node(
            package = 'robot_localization',
            name = 'ekf_filter_node',
            executable = 'ekf_node',
            parameters = [ekf]
        )
    ])

def main():

    generate_launch_description()

if __name__ == '__main__':
    main()
