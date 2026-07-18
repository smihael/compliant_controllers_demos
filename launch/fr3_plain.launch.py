# Plain FR3 bringup with one already-configured compliant controller.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    include_franka = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_bringup'),
                'launch',
                'franka.launch.py',
            ])
        ),
        launch_arguments={
            'robot_type': LaunchConfiguration('robot_type'),
            'arm_id': LaunchConfiguration('arm_id'),
            'arm_prefix': LaunchConfiguration('arm_prefix'),
            'namespace': LaunchConfiguration('namespace'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': LaunchConfiguration('load_gripper'),
            'use_fake_hardware': 'false',
            'fake_sensor_commands': 'false',
            'joint_state_rate': LaunchConfiguration('joint_state_rate'),
            'controllers_yaml': PathJoinSubstitution([
                FindPackageShare('compliant_controllers_demos'),
                'config',
                'fr3_controllers.yaml',
            ]),
        }.items(),
    )

    include_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('compliant_controllers'),
                'launch',
                'plain_controller.launch.py',
            ])
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'controller_name': LaunchConfiguration('controller_name'),
            'start_controller': LaunchConfiguration('start_controller'),
            'controller_manager_timeout': LaunchConfiguration('controller_manager_timeout'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='fr3'),
        DeclareLaunchArgument('arm_id', default_value='fr3'),
        DeclareLaunchArgument('arm_prefix', default_value=''),
        DeclareLaunchArgument('namespace', default_value='fr3'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.1'),
        DeclareLaunchArgument('load_gripper', default_value='false'),
        DeclareLaunchArgument('joint_state_rate', default_value='30'),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller'),
        DeclareLaunchArgument('start_controller', default_value='true'),
        DeclareLaunchArgument('controller_manager_timeout', default_value='30'),
        include_franka,
        include_controller,
    ])
