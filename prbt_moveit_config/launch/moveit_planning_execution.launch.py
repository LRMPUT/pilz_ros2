import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    gripper = LaunchConfiguration('gripper').perform(context)
    has_gripper = gripper != ''

    # --- Robot Description (URDF) ---
    if has_gripper:
        xacro_file = os.path.join(
            get_package_share_directory('pilz_tutorial'),
            'urdf', 'my_first_application.xacro')
        robot_description_content = xacro.process_file(
            xacro_file, mappings={'gripper': gripper}).toxml()
    else:
        xacro_file = os.path.join(
            get_package_share_directory('pilz_tutorial'),
            'urdf', 'my_first_application.xacro')
        robot_description_content = xacro.process_file(
            xacro_file, mappings={'gripper': ''}).toxml()

    robot_description = {'robot_description': robot_description_content}

    # --- SRDF ---
    srdf_xacro_file = os.path.join(
        get_package_share_directory('prbt_moveit_config'),
        'config', 'prbt.srdf.xacro')
    srdf_content = xacro.process_file(
        srdf_xacro_file, mappings={'gripper': gripper}).toxml()
    robot_description_semantic = {'robot_description_semantic': srdf_content}

    # --- Kinematics ---
    kinematics_yaml_raw = load_yaml('prbt_moveit_config', 'config/kinematics.yaml')
    kinematics_yaml = {'robot_description_kinematics': kinematics_yaml_raw}

    # --- Joint limits ---
    if has_gripper:
        joint_limits_yaml = load_yaml('prbt_pg70_support', 'config/joint_limits.yaml')
    else:
        joint_limits_yaml = load_yaml('prbt_moveit_config', 'config/joint_limits.yaml')

    # --- Cartesian limits ---
    cartesian_limits_yaml = load_yaml('prbt_moveit_config', 'config/cartesian_limits.yaml')

    # --- Planning pipeline: Pilz Industrial Motion Planner ---
    pilz_planning_yaml = load_yaml('prbt_moveit_config', 'config/pilz_industrial_motion_planner_planning.yaml')

    planning_pipeline_config = {
        'planning_pipelines': ['pilz_industrial_motion_planner'],
        'default_planning_pipeline': 'pilz_industrial_motion_planner',
        'pilz_industrial_motion_planner': pilz_planning_yaml,
    }

    # MoveGroup capabilities (loaded as move_group plugins)
    move_group_capabilities = {
        'capabilities': 'pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService',
    }

    # --- Trajectory execution ---
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # --- Controller manager ---
    if has_gripper:
        moveit_controllers_yaml = load_yaml('prbt_moveit_config', 'config/moveit_controllers_with_gripper.yaml')
        ros2_controllers_file = os.path.join(
            get_package_share_directory('prbt_moveit_config'),
            'config', 'ros2_controllers_with_gripper.yaml')
    else:
        moveit_controllers_yaml = load_yaml('prbt_moveit_config', 'config/moveit_controllers.yaml')
        ros2_controllers_file = os.path.join(
            get_package_share_directory('prbt_moveit_config'),
            'config', 'ros2_controllers.yaml')

    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    moveit_controllers.update(moveit_controllers_yaml)

    # --- move_group node ---
    move_group_params = {}
    move_group_params.update(robot_description)
    move_group_params.update(robot_description_semantic)
    move_group_params.update(kinematics_yaml)
    move_group_params.update({'robot_description_planning': {**joint_limits_yaml, **cartesian_limits_yaml}})
    move_group_params.update(planning_pipeline_config)
    move_group_params.update(move_group_capabilities)
    move_group_params.update(trajectory_execution)
    move_group_params.update(moveit_controllers)
    move_group_params.update(planning_scene_monitor_parameters)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[move_group_params],
    )

    # --- ros2_control controller_manager ---
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_controllers_file],
        output='screen',
    )

    # --- Joint state broadcaster ---
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # --- Joint trajectory controller ---
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # --- Robot state publisher ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # --- RViz ---
    rviz_config_file = os.path.join(
        get_package_share_directory('prbt_moveit_config'),
        'config', 'moveit.rviz')

    rviz_params = {}
    rviz_params.update(robot_description)
    rviz_params.update(robot_description_semantic)
    rviz_params.update(kinematics_yaml)
    rviz_params.update(planning_pipeline_config)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[rviz_params],
    )

    nodes_to_start = [
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ]

    # Gripper controller spawner
    if has_gripper:
        gripper_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )
        nodes_to_start.append(gripper_controller_spawner)

    return nodes_to_start


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gripper', default_value='', description='Gripper type (e.g. pg70)'),
        OpaqueFunction(function=launch_setup),
    ])
