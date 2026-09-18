"""Real FR3 in /fr3, with stock bringup or an optional mounted-tool description."""

import json
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, Shutdown)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def include(package, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename,
        ])),
        launch_arguments=arguments.items(),
    )


def arm(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    description_file = value('description_file')
    if not description_file:
        # Preserve upstream behavior, including the optional stock Franka hand.
        return [include('franka_bringup', 'franka.launch.py', {
            'robot_type': 'fr3',
            'namespace': 'fr3',
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': LaunchConfiguration('load_gripper'),
            'use_fake_hardware': 'false',
            'fake_sensor_commands': 'false',
            'controllers_yaml': PathJoinSubstitution([
                FindPackageShare('franka_real'), 'config', 'fr3_controllers.yaml',
            ]),
        })]

    if value('load_gripper').lower() == 'true':
        raise ValueError('Custom descriptions must launch their own gripper driver; set load_gripper:=false')
    mappings = json.loads(value('description_mappings'))
    if not isinstance(mappings, dict) or not all(
            isinstance(k, str) and isinstance(v, str) for k, v in mappings.items()):
        raise ValueError('description_mappings must be a JSON object of string xacro arguments')
    sources = json.loads(value('extra_joint_state_sources'))
    if not isinstance(sources, list) or not all(isinstance(item, str) for item in sources):
        raise ValueError('extra_joint_state_sources must be a JSON list of topic names')
    description = xacro.process_file(description_file, mappings={
        **mappings, 'robot_ip': value('robot_ip'), 'ros2_control': 'true',
        'use_fake_hardware': 'false',
    }).toxml()
    document = xacro.parse(description)
    if value('ee_frame') not in {link.getAttribute('name') for link in document.getElementsByTagName('link')}:
        raise ValueError('ee_frame must name a link in the mounted robot description')

    runtime = tempfile.TemporaryDirectory(prefix='franka-real-')
    arm_config = os.path.join(runtime.name, 'arm.yaml')
    with open(os.path.join(get_package_share_directory('franka_real'),
                           'config', 'fr3_controllers.yaml'), encoding='utf-8') as stream:
        params = yaml.safe_load(stream)
    params['/**']['franka_robot_state_broadcaster']['ros__parameters']['robot_description'] = description
    with open(arm_config, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(params, stream)

    def cleanup(_context):
        runtime.cleanup()
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', namespace='fr3',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node', namespace='fr3',
             parameters=[arm_config, {'robot_description': description, 'robot_type': 'fr3',
                                      'load_gripper': False, 'arm_prefix': ''}],
             remappings=[('joint_states', 'arm/joint_states')],
             on_exit=Shutdown(), output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             namespace='fr3', parameters=[{
                 'source_list': ['arm/joint_states'] + sources,
                 'rate': 30, 'use_robot_description': False,
             }], output='screen'),
        Node(package='controller_manager', executable='spawner', namespace='fr3',
             arguments=['joint_state_broadcaster', '-c', '/fr3/controller_manager'], output='screen'),
        Node(package='controller_manager', executable='spawner', namespace='fr3',
             arguments=['franka_robot_state_broadcaster', '-c', '/fr3/controller_manager',
                        '--param-file', arm_config], output='screen'),
    ]


def controller(context):
    name = LaunchConfiguration('controller_name').perform(context)
    joint = name == 'joint_impedance_controller'
    arguments = {
        'namespace': 'fr3',
        'arm_id': 'fr3',
        'ee_frame': LaunchConfiguration('ee_frame'),
        'controller_name': name,
        'robot_description_node': 'robot_state_publisher',
        'robot_profile_file': LaunchConfiguration('robot_profile_file'),
        'friction_compensation_profile': LaunchConfiguration('friction_compensation_profile'),
        'friction_compensation_enabled': LaunchConfiguration('friction_compensation_enabled'),
        'impl_library': 'libjoint_impedance_impl.so' if joint else 'libcartesian_impedance_impl.so',
    }
    if not joint:
        arguments.update(base_frame='fr3_link0', gravity_compensation_enabled='false')
    return [include('compliant_controllers',
                    'joint_wrapper.launch.py' if joint else 'cartesian_wrapper.launch.py',
                    arguments)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='Required FCI IP or hostname'),
        DeclareLaunchArgument('load_gripper', default_value='false'),
        DeclareLaunchArgument('description_file', default_value='',
                              description='Optional custom xacro file; empty uses upstream Franka bringup'),
        DeclareLaunchArgument('description_mappings', default_value='{}',
                              description='JSON object of custom xacro arguments (string values)'),
        DeclareLaunchArgument('extra_joint_state_sources', default_value='[]',
                              description='JSON list of accessory joint-state topics for a custom description'),
        DeclareLaunchArgument('ee_frame', default_value='fr3_link8'),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller',
                              choices=['cartesian_impedance_controller', 'joint_impedance_controller']),
        DeclareLaunchArgument('robot_profile_file', default_value=''),
        DeclareLaunchArgument('friction_compensation_profile', default_value='friction_compensation_sigmoid'),
        DeclareLaunchArgument('friction_compensation_enabled', default_value='false'),
        OpaqueFunction(function=arm),
        OpaqueFunction(function=controller),
    ])
