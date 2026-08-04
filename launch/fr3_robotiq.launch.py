import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_robot_profile(defaults, profile_name):
    profile = defaults.copy()
    config_path = os.path.join(
        get_package_share_directory('compliant_controllers_demos'),
        'config',
        'robot_config.yaml',
    )
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            loaded = yaml.safe_load(f) or {}
        if isinstance(loaded, dict):
            selected = loaded.get(profile_name)
            if isinstance(selected, dict):
                profile.update({k: v for k, v in selected.items() if v is not None})
            else:
                profile.update({k: v for k, v in loaded.items() if v is not None})
    except Exception:
        pass
    return profile


def generate_launch_description():
    robot_profile = os.environ.get('COMPLIANT_ROBOT_PROFILE', 'ROBOT_1')
    robot_cfg = load_robot_profile({
        'robot_name': 'fr3',
        'robot_ip': '192.168.1.1',
    }, robot_profile)

    robotiq_model = PathJoinSubstitution([
        FindPackageShare('compliant_controllers_demos'),
        'urdf',
        'fr3_robotiq_2f_85.urdf.xacro',
    ])

    declared_args = [
        DeclareLaunchArgument('robot_profile', default_value=robot_profile),
        DeclareLaunchArgument('robot_profile_file', default_value=''),
        DeclareLaunchArgument(
            'friction_compensation_profile',
            default_value='friction_compensation_sigmoid'),
        DeclareLaunchArgument('robot_type', default_value='fr3'),
        DeclareLaunchArgument('arm_id', default_value=str(robot_cfg.get('robot_name', 'fr3'))),
        DeclareLaunchArgument('arm_prefix', default_value=''),
        DeclareLaunchArgument('namespace', default_value='fr3'),
        DeclareLaunchArgument('robot_ip', default_value=str(robot_cfg.get('robot_ip', '192.168.1.1'))),
        DeclareLaunchArgument('no_prefix', default_value='false'),
        DeclareLaunchArgument('robotiq_prefix', default_value=''),
        DeclareLaunchArgument('robotiq_parent', default_value=''),
        DeclareLaunchArgument('xyz_robotiq', default_value='0 0 0'),
        DeclareLaunchArgument('rpy_robotiq', default_value='0 0 0'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false'),
        DeclareLaunchArgument('launch_robotiq_server', default_value='true'),
        DeclareLaunchArgument('com_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('joint_state_rate', default_value='30'),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller'),
        DeclareLaunchArgument('impl_library', default_value=''),
        DeclareLaunchArgument('ee_frame', default_value=''),
        DeclareLaunchArgument('base_frame', default_value='base'),
        DeclareLaunchArgument('dithering_enabled', default_value=''),
        DeclareLaunchArgument('friction_compensation_enabled', default_value=''),
        DeclareLaunchArgument('max_step_guard_enabled', default_value=''),
        DeclareLaunchArgument('plugin_params_file', default_value=''),
        DeclareLaunchArgument('log_file', default_value=''),
        DeclareLaunchArgument('shutdown_on_done', default_value='false'),
        DeclareLaunchArgument('publish_world_to_base', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([
            FindPackageShare('franka_description'),
            'rviz',
            'visualize_franka.rviz',
        ])),
        DeclareLaunchArgument('fixed_frame', default_value='fr3_link0'),
    ]

    include_fr3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('compliant_controllers_demos'),
                'launch',
                'fr3.launch.py',
            ])
        ),
        launch_arguments={
            'robot_profile': LaunchConfiguration('robot_profile'),
            'robot_profile_file': LaunchConfiguration('robot_profile_file'),
            'friction_compensation_profile': LaunchConfiguration('friction_compensation_profile'),
            'robot_type': LaunchConfiguration('robot_type'),
            'arm_id': LaunchConfiguration('arm_id'),
            'arm_prefix': LaunchConfiguration('arm_prefix'),
            'namespace': LaunchConfiguration('namespace'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': 'false',
            'joint_state_rate': LaunchConfiguration('joint_state_rate'),
            'controller_name': LaunchConfiguration('controller_name'),
            'impl_library': LaunchConfiguration('impl_library'),
            'ee_frame': LaunchConfiguration('ee_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'dithering_enabled': LaunchConfiguration('dithering_enabled'),
            'friction_compensation_enabled': LaunchConfiguration('friction_compensation_enabled'),
            'max_step_guard_enabled': LaunchConfiguration('max_step_guard_enabled'),
            'plugin_params_file': LaunchConfiguration('plugin_params_file'),
            'log_file': LaunchConfiguration('log_file'),
            'shutdown_on_done': LaunchConfiguration('shutdown_on_done'),
            'use_rviz': 'false',
            'publish_world_to_base': LaunchConfiguration('publish_world_to_base'),
        }.items(),
    )

    include_robotiq = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robotiq_description'),
                'launch',
                'robotiq_control.launch.py',
            ])
        ),
        launch_arguments={
            'model': robotiq_model,
            'launch_rviz': 'false',
            'com_port': LaunchConfiguration('com_port'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_robotiq_server')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '--display-config',
            LaunchConfiguration('rviz_config'),
            '-f',
            LaunchConfiguration('fixed_frame'),
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription(declared_args + [
        include_fr3,
        include_robotiq,
        rviz,
    ])
