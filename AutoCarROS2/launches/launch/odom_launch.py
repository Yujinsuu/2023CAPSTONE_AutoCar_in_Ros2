import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():

    odom = 'autocar_odom'
    gzpkg = 'autocar_gazebo'
    descpkg = 'autocar_description'

    world = os.path.join(get_package_share_directory(gzpkg), 'worlds', 'odom.world')
    urdf = os.path.join(get_package_share_directory(descpkg),'urdf', 'autocar.xacro')
    rviz = os.path.join(get_package_share_directory(descpkg), 'rviz', 'view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    subprocess.run(['killall', 'gzserver'])
    subprocess.run(['killall', 'gzclient'])

    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'
        ),

        SetEnvironmentVariable(
            'RCUTILS_COLORIZED_OUTPUT', '1'
        ),
#
        # ExecuteProcess(
            # cmd=['gzserver', '--verbose', world, 'libgazebo_ros_factory.so'],
        # ),
#
        # DeclareLaunchArgument(
            # 'use_sim_time',
            # default_value='false',
            # description='Use simulation (Gazebo) clock if true'
        # ),
#
        # Node(
            # package='robot_state_publisher',
            # name='robot_state_publisher',
            # executable='robot_state_publisher',
            # output={'both': 'log'},
            # parameters=[{'use_sim_time': use_sim_time}],
            # arguments=[urdf]
        # ),

        Node(
            package = odom,
            name = 'odom_pub',
            executable = 'odom_pub.py'
        ),

        Node(
            package = odom,
            name = 'odom_viz',
            executable = 'odom_viz.py'
            #executable = 'use_urdf.py'
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
