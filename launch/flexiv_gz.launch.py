"""Gazebo simulation launch for Flexiv EnlightL robot using compliant controllers."""

import os
import xml.etree.ElementTree as ET

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _build_robot_description(context: LaunchContext, load_gripper, use_fake_hardware,
                             fake_sensor_commands, controller_name):
    load_gripper_str = context.perform_substitution(load_gripper).lower() in ('true', '1', 'yes')
    use_fake_str = context.perform_substitution(use_fake_hardware).lower() in ('true', '1', 'yes')
    fake_sensor_str = context.perform_substitution(fake_sensor_commands).lower() in ('true', '1', 'yes')
    controller_name_str = context.perform_substitution(controller_name)

    urdf_xacro = PathJoinSubstitution([
        FindPackageShare('flexiv_description'), 'urdf', 'flexiv.urdf.xacro'
    ])

    xacro_cmd = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', urdf_xacro,
        ' robot_type:=EnlightL',
        ' robot_sn:=',
        ' ros2_control:=true',
        ' use_fake_hardware:=true',
        ' fake_sensor_commands:=', 'true' if fake_sensor_str else 'false',
        ' load_gripper:=' + ('true' if load_gripper_str else 'false'),
    ])

    urdf_str = xacro_cmd.perform(context)
    root = ET.fromstring(urdf_str)

    for plugin in root.findall('.//plugin'):
        if plugin.text == 'mock_components/GenericSystem':
            plugin.text = 'gz_ros2_control/GazeboSimSystem'

    is_joint = controller_name_str == 'joint_impedance_controller'
    for joint in root.findall('.//joint'):
        joint_name = joint.get('name', '')
        if not any(joint_name.endswith(f'joint{i}') for i in range(1, 8)):
            continue
        has_effort = any(
            ci.get('name') == 'effort'
            for ci in joint.findall('command_interface')
        )
        if not has_effort:
            pos_cmd = joint.find("command_interface[@name='position']")
            if pos_cmd is not None:
                idx = list(joint).index(pos_cmd) + 1
                ET.SubElement(joint, 'command_interface', {'name': 'effort'})

    modified_urdf = ET.tostring(root, encoding='unicode')

    return [Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='both',
        parameters=[{'robot_description': modified_urdf, 'use_sim_time': True}]
    ), modified_urdf]


def _gazebo_include(context: LaunchContext, world, show_gazebo_gui, controller_debug):
    world_file = context.perform_substitution(world)
    gui_flag = context.perform_substitution(show_gazebo_gui).lower() in ('true', '1', 'yes')
    debug_flag = context.perform_substitution(controller_debug).lower() in ('true', '1', 'yes')

    if gui_flag:
        gz_args = f'{world_file} -r'
    else:
        verbosity = '4' if debug_flag else '2'
        gz_args = f'{world_file} -v {verbosity} -s -r --headless-rendering'

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )]


_urdf_cache = {}


def _build_runtime_nodes(context: LaunchContext, show_rviz, controller_debug):
    controller_name = context.perform_substitution(LaunchConfiguration('controller_name'))
    impl_library = context.perform_substitution(LaunchConfiguration('impl_library'))
    rviz_flag = context.perform_substitution(show_rviz).lower() in ('true', '1', 'yes')

    spawn = Node(
        package='ros_gz_sim', executable='create', name='spawn_flexiv',
        arguments=['-topic', '/robot_description', '-name', 'flexiv', '-allow_renaming', 'true'],
        output='screen',
    )

    jsb_spawner = Node(
        package='controller_manager', executable='spawner', name='spawner_jsb',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    is_joint = controller_name == 'joint_impedance_controller'
    if is_joint:
        controller_launch_file = 'joint_wrapper.launch.py'
        controller_launch_arguments = {
            'namespace': '',
            'arm_id': '',
            'controller_name': controller_name,
            'controller_manager': '/controller_manager',
            'robot_description_node': '/robot_state_publisher',
            'robot_description_param': 'robot_description',
        }
    else:
        controller_launch_file = 'cartesian_wrapper.launch.py'
        controller_launch_arguments = {
            'namespace': '',
            'arm_id': '',
            'controller_name': controller_name,
            'controller_manager': '/controller_manager',
            'init_k_pos': LaunchConfiguration('init_k_pos'),
            'init_k_ori': LaunchConfiguration('init_k_ori'),
            'ee_frame': 'flange',
            'base_frame': 'base_link',
            'robot_description_node': '/robot_state_publisher',
            'robot_description_param': 'robot_description',
            'load_end_effector_profile': 'false',
            'tcp_enabled': LaunchConfiguration('tcp_enabled'),
            'gravity_compensation_enabled': LaunchConfiguration('gravity_compensation_enabled'),
            'ee_load_compensation_enabled': LaunchConfiguration('ee_load_compensation_enabled'),
            'friction_compensation_enabled': LaunchConfiguration('friction_compensation_enabled'),
            'friction_model': LaunchConfiguration('friction_model'),
            'friction_scale': LaunchConfiguration('friction_scale'),
            'friction_use_gating': LaunchConfiguration('friction_use_gating'),
            'diagnostic_log_file': LaunchConfiguration('diagnostic_log_file'),
            'diagnostic_log_duration': LaunchConfiguration('diagnostic_log_duration'),
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'publish_world_to_base': LaunchConfiguration('publish_world_to_base'),
        }

    controller_launch_arguments['impl_library'] = impl_library or (
        'libjoint_impedance_impl.so' if is_joint
        else 'libcartesian_impedance_impl.so'
    )

    include_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('compliant_controllers'),
                'launch',
                controller_launch_file,
            )
        ),
        launch_arguments=controller_launch_arguments.items(),
    )

    spawn_to_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb_spawner, include_controller])
    )

    nodes = [spawn, spawn_to_controllers]

    if rviz_flag:
        rviz_config = os.path.join(
            get_package_share_directory('flexiv_description'), 'rviz', 'view_flexiv.rviz'
        )
        if os.path.exists(rviz_config):
            nodes.append(Node(
                package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', rviz_config],
            ))

    return nodes


def _resolve_robot_description(context: LaunchContext, load_gripper, use_fake_hardware,
                                fake_sensor_commands, controller_name):
    nodes_and_urdf = _build_robot_description(
        context, load_gripper, use_fake_hardware, fake_sensor_commands, controller_name
    )
    _urdf_cache['current'] = nodes_and_urdf[1]
    return nodes_and_urdf[:1]


def generate_launch_description():
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('flexiv_description')
    )

    declared_args = [
        DeclareLaunchArgument('load_gripper', default_value='false',
                              description='Load gripper in URDF'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                              description='Use mock hardware (for Gazebo we override to true)'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false',
                              description='Enable fake sensor commands'),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller',
                              description='Primary controller to spawn'),
        DeclareLaunchArgument('impl_library', default_value=''),
        DeclareLaunchArgument('init_k_pos', default_value='200.0'),
        DeclareLaunchArgument('init_k_ori', default_value='10.0'),
        DeclareLaunchArgument('tcp_enabled', default_value='false'),
        DeclareLaunchArgument('gravity_compensation_enabled', default_value='true'),
        DeclareLaunchArgument('ee_load_compensation_enabled', default_value='false'),
        DeclareLaunchArgument('friction_compensation_enabled', default_value='false'),
        DeclareLaunchArgument('friction_model', default_value='auto'),
        DeclareLaunchArgument('friction_scale', default_value='1.0'),
        DeclareLaunchArgument('friction_use_gating', default_value='true'),
        DeclareLaunchArgument('diagnostic_log_file', default_value=''),
        DeclareLaunchArgument('diagnostic_log_duration', default_value='0.0'),
        DeclareLaunchArgument('diagnostic_mode', default_value='0'),
        DeclareLaunchArgument('world', default_value='empty.sdf',
                              description='Gazebo world file'),
        DeclareLaunchArgument('show_gazebo_gui', default_value='false',
                              description='Show Gazebo GUI'),
        DeclareLaunchArgument('show_rviz', default_value='true',
                              description='Launch RViz'),
        DeclareLaunchArgument('publish_world_to_base', default_value='true',
                              description='Publish static world->base TF'),
        DeclareLaunchArgument('controller_debug', default_value='false',
                              description='Enable debug logging'),
    ]

    load_gripper = LaunchConfiguration('load_gripper')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    controller_name = LaunchConfiguration('controller_name')
    world = LaunchConfiguration('world')
    show_gazebo_gui = LaunchConfiguration('show_gazebo_gui')
    controller_debug = LaunchConfiguration('controller_debug')
    show_rviz = LaunchConfiguration('show_rviz')

    set_controller_debug = SetEnvironmentVariable(
        name='CONTROLLER_DEBUG', value=controller_debug
    )

    gazebo = OpaqueFunction(
        function=_gazebo_include, args=[world, show_gazebo_gui, controller_debug]
    )

    robot_description = OpaqueFunction(
        function=_resolve_robot_description,
        args=[load_gripper, use_fake_hardware, fake_sensor_commands, controller_name],
    )

    runtime_nodes = OpaqueFunction(
        function=_build_runtime_nodes, args=[show_rviz, controller_debug],
    )

    return LaunchDescription(
        declared_args + [
            set_controller_debug,
            gazebo,
            robot_description,
            runtime_nodes,
        ]
    )
