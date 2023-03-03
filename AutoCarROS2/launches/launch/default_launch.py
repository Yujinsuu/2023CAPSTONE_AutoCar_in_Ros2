import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():

    navpkg = 'autocar_nav'
    mappkg = 'autocar_route'

    navconfig = os.path.join(get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')

    # 962730.0368901521;1959367.9756638794;0.0

    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'
        ),

        SetEnvironmentVariable(
            'RCUTILS_COLORIZED_OUTPUT', '1'
        ),

        Node(
            package = navpkg,
            name = 'localisation',
            executable = 'localisation.py',
            parameters = [navconfig]
        ),

        Node(
            package = navpkg,
            name = 'global_map',
            executable = 'map_visualizer.py'
        ),

        Node(
            package = navpkg,
            name = 'global_planner',
            executable = 'globalplanner.py',
            parameters = [navconfig]
        ),

        Node(
            package = navpkg,
            name = 'local_planner',
            executable = 'localplanner.py',
            parameters = [navconfig]
        ),

        Node(
            package = mappkg,
            name = 'bof',
            executable = 'bof',
        ),

        Node(
            package = navpkg,
            name = 'path_tracker',
            executable = 'tracker.py',
            parameters = [navconfig]
        )
    ])

def main():

    generate_launch_description()

if __name__ == '__main__':
    main()
