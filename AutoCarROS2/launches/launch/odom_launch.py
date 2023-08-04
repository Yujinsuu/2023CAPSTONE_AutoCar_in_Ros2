import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    odom = 'autocar_odom'
    mappkg = 'autocar_map'
    navpkg = 'autocar_nav'

    navconfig = os.path.join(get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')
    rviz = os.path.join(get_package_share_directory(odom), 'rviz', 'view.rviz')
    ekf = os.path.join(get_package_share_directory(odom), 'config', 'ekf.yaml')


    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'
        ),

        SetEnvironmentVariable(
            'RCUTILS_COLORIZED_OUTPUT', '1'
        ),

        # Node(
        #     package = 'ros1_bridge',
        #     name = 'ros1_bridge',
        #     executable = 'parameter_bridge'
        # ),

        # Node(
        #     package = 'bluespace_ai_xsens_mti_driver',
        #     name = 'xsens_driver',
        #     executable = 'xsens_mti_node'
        # ),

        Node(
            package = odom,
            name = 'odom_pub',
            executable = 'odom_pub.py'
        ),

        # Node(
        #     package = odom,
        #     name = 'pub_encoder_tic',
        #     executable = 'get_encoder.py'
        # ),

        # Node(
        #     package = odom,
        #     name = 'pub_enc_vel',
        #     executable = 'encoder_vel.py'
        # ),

        Node(
            package = 'robot_localization',
            name = 'ekf_filter_node',
            executable = 'ekf_node',
            parameters = [ekf]
        ),

        # Node(
        #     package = odom,
        #     name = 'dead_reckoning',
        #     executable = 'dead_reckoning.py'
        # ),

        Node(
            package = navpkg,
            name = 'localization',
            executable = 'localization.py',
            parameters = [navconfig]
        ),

        Node(
            package = odom,
            name = 'odom_viz',
            executable = 'odom_viz.py'
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
    ])

def main():

    generate_launch_description()

if __name__ == '__main__':
    main()
