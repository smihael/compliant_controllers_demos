"""FR3 SpaceMouse teleop demo for compliant_controllers CartesianCommand input."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('launch_robot', default_value='true'),
        DeclareLaunchArgument('robot_type', default_value='fr3'),
        DeclareLaunchArgument('arm_id', default_value='fr3'),
        DeclareLaunchArgument('namespace', default_value='fr3'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.1'),
        DeclareLaunchArgument('load_gripper', default_value='false'),
        DeclareLaunchArgument('joint_state_rate', default_value='30'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('publish_world_to_base', default_value='true'),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller'),
        DeclareLaunchArgument('device_path', default_value=''),
        DeclareLaunchArgument('operator_position_front', default_value='false'),
        DeclareLaunchArgument('base_frame', default_value='fr3_link0'),
        DeclareLaunchArgument('ee_frame', default_value='fr3_link8'),
        DeclareLaunchArgument('linear_scale', default_value='0.08'),
        DeclareLaunchArgument('angular_scale', default_value='0.35'),
        DeclareLaunchArgument('deadband', default_value='0.04'),
        DeclareLaunchArgument('k_lin', default_value='250.0'),
        DeclareLaunchArgument('k_rot', default_value='18.0'),
        DeclareLaunchArgument('damping_ratio', default_value='1.0'),
    ]

    include_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('compliant_controllers_demos'),
                'launch',
                'fr3.launch.py',
            ])
        ),
        launch_arguments={
            'robot_type': LaunchConfiguration('robot_type'),
            'arm_id': LaunchConfiguration('arm_id'),
            'namespace': LaunchConfiguration('namespace'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': LaunchConfiguration('load_gripper'),
            'joint_state_rate': LaunchConfiguration('joint_state_rate'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'publish_world_to_base': LaunchConfiguration('publish_world_to_base'),
            'controller_name': LaunchConfiguration('controller_name'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_robot')),
    )

    spacemouse = Node(
        package='spacemouse_publisher',
        executable='pyspacemouse_publisher',
        name='spacemouse_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'operator_position_front': ParameterValue(LaunchConfiguration('operator_position_front'), value_type=bool),
            'device_path': LaunchConfiguration('device_path'),
        }],
    )

    bridge = Node(
        package='compliant_controllers_demos',
        executable='spacemouse_to_cartesian_command.py',
        name='spacemouse_to_cartesian_command',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'input_topic': 'franka_controller/target_cartesian_velocity_percent',
            'output_topic': 'cartesian_command',
            'base_frame': LaunchConfiguration('base_frame'),
            'ee_frame': LaunchConfiguration('ee_frame'),
            'linear_scale': ParameterValue(LaunchConfiguration('linear_scale'), value_type=float),
            'angular_scale': ParameterValue(LaunchConfiguration('angular_scale'), value_type=float),
            'deadband': ParameterValue(LaunchConfiguration('deadband'), value_type=float),
            'k_lin': ParameterValue(LaunchConfiguration('k_lin'), value_type=float),
            'k_rot': ParameterValue(LaunchConfiguration('k_rot'), value_type=float),
            'damping_ratio': ParameterValue(LaunchConfiguration('damping_ratio'), value_type=float),
        }],
    )

    return LaunchDescription(declared_args + [include_robot, spacemouse, bridge])
