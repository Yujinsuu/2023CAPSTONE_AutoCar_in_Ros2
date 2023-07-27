import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():

    odom = 'autocar_odom'
    mappkg = 'autocar_map'
    navpkg = 'autocar_nav'

    rviz = os.path.join(get_package_share_directory(odom), 'rviz', 'view.rviz')
    navconfig = os.path.join(get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')


    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'
        ),

        SetEnvironmentVariable(
            'RCUTILS_COLORIZED_OUTPUT', '1'
        ),

        Node(
            package = navpkg,
            name = 'simulation',
            executable = 'simulation.py',
            parameters = [navconfig]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],
            output={'both': 'log'}
        ),

        Node(
            package = mappkg,
            name = 'mapviz',
            executable = 'link_visualizer.py'
        ),

        Node(
            package = navpkg,
            name = 'global_planner',
            executable = 'globalplanner.py',
            parameters = [navconfig]
        ),

        Node(
            package = navpkg,
            name = 'parking_path',
            executable = 'parking_path.py'
        ),

        Node(
            package = navpkg,
            name = 'local_planner',
            executable = 'localplanner.py',
            parameters = [navconfig]
        ),

        Node(
            package = navpkg,
            name = 'path_tracker',
            executable = 'tracker.py',
            parameters = [navconfig]
        ),

        Node(
            package = navpkg,
            name = 'Core',
            executable = 'core.py'
        )
    ])

def main():

    generate_launch_description()

if __name__ == '__main__':
    main()
