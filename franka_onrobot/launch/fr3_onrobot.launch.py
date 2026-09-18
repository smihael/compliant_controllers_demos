"""onrobot tools on the shared franka_real arm bringup."""
import json
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def configure_description(context):
    mappings = {name: LaunchConfiguration(name).perform(context) for name in (
        'xyz_onrobot', 'rpy_onrobot',
    )}
    mappings['include_ros2_control'] = 'false'
    sources = []
    return [
        SetLaunchConfiguration('description_mappings', json.dumps(mappings)),
        SetLaunchConfiguration('extra_joint_state_sources', json.dumps(sources)),
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('robot_ip', description='Required FCI IP or hostname'),
        DeclareLaunchArgument('onrobot_ip_address', description='Required OnRobot sensor address'),
        DeclareLaunchArgument('onrobot_topic_name', default_value='wrench'),
        DeclareLaunchArgument('onrobot_port', default_value='49152'),
        DeclareLaunchArgument('onrobot_samples_per_request', default_value='10'),
        DeclareLaunchArgument('onrobot_speed', default_value='10'),
        DeclareLaunchArgument('onrobot_filter', default_value='4'),
        DeclareLaunchArgument('onrobot_bias_on_start', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('xyz_onrobot', default_value='0 0 0'),
        DeclareLaunchArgument('rpy_onrobot', default_value='0 0 -1.5707963267948966'),
    ]
    return LaunchDescription(arguments + [
        OpaqueFunction(function=configure_description),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_real'), 'launch', 'fr3.launch.py',
        ])), launch_arguments={
            'description_file': PathJoinSubstitution([
                FindPackageShare('franka_onrobot'), 'urdf', 'fr3_onrobot_ft.urdf.xacro',
            ]),
            'load_gripper': 'false',
            'robot_ip': LaunchConfiguration('robot_ip'),
        }.items()),
        Node(package='onrobot_ft_ros2', executable='onrobot_ft_udp_node',
             name='onrobot_ft_udp_node', namespace='fr3', output='screen',
             parameters=[{
                 'ip_address': LaunchConfiguration('onrobot_ip_address'),
                 'sensor_id': 'onrobot_fts_link', 'topic_name': LaunchConfiguration('onrobot_topic_name'),
                 'port': ParameterValue(LaunchConfiguration('onrobot_port'), value_type=int),
                 'samples_per_request': ParameterValue(LaunchConfiguration('onrobot_samples_per_request'), value_type=int),
                 'speed': ParameterValue(LaunchConfiguration('onrobot_speed'), value_type=int),
                 'filter': ParameterValue(LaunchConfiguration('onrobot_filter'), value_type=int),
                 'bias_on_start': ParameterValue(LaunchConfiguration('onrobot_bias_on_start'), value_type=bool),
             }]),
    ])
