import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gripper_arg = DeclareLaunchArgument(
        'gripper',
        default_value='',
        description='Gripper type (e.g. pg70)'
    )

    # Include the main moveit planning execution launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('prbt_moveit_config'),
                'launch',
                'moveit_planning_execution.launch.py'
            )
        ),
        launch_arguments={'gripper': LaunchConfiguration('gripper')}.items(),
    )

    return LaunchDescription([
        gripper_arg,
        moveit_launch,
    ])
