"""robotiq tools on the shared franka_real arm bringup."""
import json
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetLaunchConfiguration)
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def configure_description(context):
    mappings = {name: LaunchConfiguration(name).perform(context) for name in (
        'xyz_robotiq', 'rpy_robotiq', 'com_port',
    )}
    mappings['include_ros2_control'] = 'false'
    sources = ['robotiq/joint_states'] if LaunchConfiguration('start_robotiq').perform(context) == 'true' else []
    return [
        SetLaunchConfiguration('description_mappings', json.dumps(mappings)),
        SetLaunchConfiguration('extra_joint_state_sources', json.dumps(sources)),
    ]


def gripper(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    if value('start_robotiq') != 'true':
        return []
    # Keep serial gripper hardware out of the arm's 1 kHz control loop.
    description = xacro.process_file(value('description_file'), mappings={
        **json.loads(value('description_mappings')), 'robot_ip': value('robot_ip'),
        'ros2_control': 'false', 'include_ros2_control': 'true', 'use_fake_hardware': 'false',
    }).toxml()
    runtime = tempfile.TemporaryDirectory(prefix='robotiq-')
    config = os.path.join(runtime.name, 'robotiq.yaml')
    with open(os.path.join(get_package_share_directory('robotiq_description'),
                           'config', 'robotiq_controllers.yaml'), encoding='utf-8') as stream:
        params = yaml.safe_load(stream)
    with open(config, 'w', encoding='utf-8') as stream:
        yaml.safe_dump({f'/fr3/robotiq/{name}': settings for name, settings in params.items()}, stream)

    def cleanup(_context):
        runtime.cleanup()
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='controller_manager', executable='ros2_control_node', namespace='fr3/robotiq',
             parameters=[config, {'robot_description': description}],
             on_exit=Shutdown(), output='screen'),
        *[Node(package='controller_manager', executable='spawner', namespace='fr3/robotiq',
               arguments=[name, '-c', '/fr3/robotiq/controller_manager'], output='screen')
          for name in ('joint_state_broadcaster', 'robotiq_gripper_controller', 'robotiq_activation_controller')],
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('robot_ip', description='Required FCI IP or hostname'),
        DeclareLaunchArgument('start_robotiq', default_value='true', choices=['true', 'false'],
                              description='Start the serial gripper drivers; the mounted tool stays in the URDF'),
        DeclareLaunchArgument('com_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('xyz_robotiq', default_value='0 0 0'),
        DeclareLaunchArgument('rpy_robotiq', default_value='0 0 0'),
    ]
    return LaunchDescription(arguments + [
        OpaqueFunction(function=configure_description),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_real'), 'launch', 'fr3.launch.py',
        ])), launch_arguments={
            'description_file': PathJoinSubstitution([
                FindPackageShare('franka_robotiq'), 'urdf', 'fr3_robotiq_2f_85.urdf.xacro',
            ]),
            'load_gripper': 'false',
            'robot_ip': LaunchConfiguration('robot_ip'),
        }.items()),
        OpaqueFunction(function=gripper),
    ])
