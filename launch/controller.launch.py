from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_frame', default_value='robot1',
            description='Target frame name.'
        ),
        Node(
            package='ff_test_controllers',
            executable='setpoint_PID_controller',
            name='controller1'
        ),
        Node(
            package='ff_test_controllers',
            executable='ff_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'ff_name': 'snap'}
            ]
        ),
    ])
