from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ff_controllers',
            namespace='ff_controllers',
            executable='ff_controller',
            name='controller1',
            output='screen'
            emulate_tty=True,
            parameters=[{'controller': 'HJI'}]
        ),
        Node(
            package='px4_mpc',
            namespace='px4_mpc',
            executable='rviz_pos_marker',
            name='rviz_pos_marker',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='visualizer',
            name='visualizer'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(get_package_share_directory('px4_mpc'), 'config.rviz')]]
        )
    ])
