from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='prbt_hardware_support',
            executable='fake_speed_override_node',
            name='fake_speed_override_node',
            output='screen',
        ),
    ])
