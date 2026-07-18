"""Gazebo simulation launch for FR3 robot using compliant controllers.

Mirrors real robot launch style while adding a debug flag and optional world->base TF.
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory

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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', arm_id_str, f'{arm_id_str}.urdf.xacro'
    )
    xacro_doc = xacro.process_file(
        franka_xacro,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true',
        }
    )
    default_yaml = os.path.join(
        get_package_share_directory('franka_gazebo_bringup'),
        'config', 'franka_gazebo_controllers.yaml'
    )
    custom_yaml = os.path.join(
        get_package_share_directory('compliant_controllers_demos'),
        'config', 'fr3_gz_controllers.yaml'
    )
    urdf_xml = xacro_doc.toxml().replace(default_yaml, custom_yaml)
    return [Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
        output='both', parameters=[{'robot_description': urdf_xml}]
    )]


def _gazebo_include(context: LaunchContext, world, show_gazebo_gui, controller_debug):
    world_file = context.perform_substitution(world)
    gui_flag = context.perform_substitution(show_gazebo_gui).lower() in ('true', '1', 'yes')
    debug_flag = context.perform_substitution(controller_debug).lower() in ('true', '1', 'yes')

    if gui_flag:
        gz_args = f"{world_file} -r"
    else:
        verbosity = '4' if debug_flag else '2'
        gz_args = f"{world_file} -v {verbosity} -s -r --headless-rendering"

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    return [gazebo_launch]


def _is_joint_controller(controller_name):
    return controller_name == 'joint_impedance_controller'


def _build_runtime_nodes(context: LaunchContext, show_rviz, controller_debug):
    namespace = context.perform_substitution(LaunchConfiguration('namespace'))
    arm_id = context.perform_substitution(LaunchConfiguration('arm_id'))
    controller_manager = f'/{namespace}/controller_manager' if namespace else '/controller_manager'
    rviz_flag = context.perform_substitution(show_rviz).lower() in ('true', '1', 'yes')
    controller_name = context.perform_substitution(LaunchConfiguration('controller_name'))
    impl_library = context.perform_substitution(LaunchConfiguration('impl_library'))

    spawn = Node(
        package='ros_gz_sim', executable='create', name='spawn_fr3',
        arguments=['-topic', '/robot_description'], output='screen'
    )

    jsb_spawner = Node(
        package='controller_manager', executable='spawner', name='spawner_jsb',
        namespace=LaunchConfiguration('namespace'), output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', controller_manager],
    )

    if _is_joint_controller(controller_name):
        controller_launch_file = 'joint_wrapper.launch.py'
        controller_launch_arguments = {
            'namespace': LaunchConfiguration('namespace'),
            'arm_id': LaunchConfiguration('arm_id'),
            'controller_name': LaunchConfiguration('controller_name'),
            'controller_manager': controller_manager,
            'ee_frame': LaunchConfiguration('ee_frame'),
            'robot_description_node': '/robot_state_publisher',
            'robot_description_param': 'robot_description',
            'friction_compensation_enabled': LaunchConfiguration('friction_compensation_enabled'),
            'friction_model': LaunchConfiguration('friction_model'),
            'friction_scale': LaunchConfiguration('friction_scale'),
            'friction_use_gating': LaunchConfiguration('friction_use_gating'),
            'initial_stiffness': LaunchConfiguration('joint_initial_stiffness'),
            'initial_damping': LaunchConfiguration('joint_initial_damping'),
            'filter_alpha': LaunchConfiguration('joint_filter_alpha'),
            'max_tau_delta': LaunchConfiguration('joint_max_tau_delta'),
            'power_enable_tau_norm_threshold': LaunchConfiguration('joint_power_enable_tau_norm_threshold'),
            'max_power_enable_count': LaunchConfiguration('joint_max_power_enable_count'),
        }
    else:
        controller_launch_file = 'cartesian_wrapper.launch.py'
        controller_launch_arguments = {
            'namespace': LaunchConfiguration('namespace'),
            'arm_id': LaunchConfiguration('arm_id'),
            'controller_name': LaunchConfiguration('controller_name'),
            'controller_manager': controller_manager,
            'init_k_pos': LaunchConfiguration('init_k_pos'),
            'init_k_ori': LaunchConfiguration('init_k_ori'),
            'ee_frame': LaunchConfiguration('ee_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'robot_description_node': '/robot_state_publisher',
            'robot_description_param': 'robot_description',
            'gravity_compensation_enabled': LaunchConfiguration('gravity_compensation_enabled'),
            'ee_load_compensation_enabled': LaunchConfiguration('ee_load_compensation_enabled'),
            'friction_compensation_enabled': LaunchConfiguration('friction_compensation_enabled'),
            'friction_model': LaunchConfiguration('friction_model'),
            'friction_scale': LaunchConfiguration('friction_scale'),
            'friction_use_gating': LaunchConfiguration('friction_use_gating'),
            'diagnostic_log_file': LaunchConfiguration('diagnostic_log_file'),
            'diagnostic_log_duration': LaunchConfiguration('diagnostic_log_duration'),
            'diagnostic_log_filter_tag': LaunchConfiguration('diagnostic_log_filter_tag'),
            'publish_world_to_base': LaunchConfiguration('publish_world_to_base'),
        }
    controller_launch_arguments['impl_library'] = impl_library or (
        'libjoint_impedance_impl.so' if _is_joint_controller(controller_name)
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

    # Start state broadcaster and primary controller in parallel right after spawn
    # to minimize the no-controller startup window (reduces initial sag).
    spawn_to_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb_spawner, include_controller])
    )

    nodes = [spawn, spawn_to_controllers]

    # RViz
    if rviz_flag:
        rviz_config = os.path.join(get_package_share_directory('franka_description'), 'rviz', 'visualize_franka.rviz')
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            namespace=LaunchConfiguration('namespace'), output='screen',
            arguments=['--display-config', rviz_config, '-f', 'world']
        ))

    # Joint state publisher
    #nodes.append(Node(
    #    package='joint_state_publisher', executable='joint_state_publisher', name='joint_state_publisher',
    #    parameters=[{'source_list': ['joint_states'], 'rate': 30}], output='screen'
    #))

    return nodes


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('arm_id', default_value='fr3', description='Arm identifier'),
        DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
        DeclareLaunchArgument('load_gripper', default_value='false', description='Load gripper in URDF'),
        DeclareLaunchArgument('franka_hand', default_value='franka_hand', description='Gripper variant'),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller', description='Primary controller to spawn'),
        DeclareLaunchArgument('impl_library', default_value=''),
        DeclareLaunchArgument('init_k_pos', default_value='200.0'),
        DeclareLaunchArgument('init_k_ori', default_value='10.0'),
        DeclareLaunchArgument('joint_initial_stiffness', default_value='600,600,600,600,250,150,50'),
        DeclareLaunchArgument('joint_initial_damping', default_value='30,30,30,30,10,10,5'),
        DeclareLaunchArgument('joint_filter_alpha', default_value='0.99'),
        DeclareLaunchArgument('joint_max_tau_delta', default_value='1.0'),
        DeclareLaunchArgument('joint_power_enable_tau_norm_threshold', default_value='1.1'),
        DeclareLaunchArgument('joint_max_power_enable_count', default_value='100'),
        DeclareLaunchArgument('ee_frame', default_value=''),
        DeclareLaunchArgument('base_frame', default_value=''),
        DeclareLaunchArgument('gravity_compensation_enabled', default_value='true'),
        DeclareLaunchArgument('ee_load_compensation_enabled', default_value='false'),
        DeclareLaunchArgument('friction_compensation_enabled', default_value='false'),
        DeclareLaunchArgument('friction_model', default_value='auto'),
        DeclareLaunchArgument('friction_scale', default_value='1.0'),
        DeclareLaunchArgument('friction_use_gating', default_value='true'),
        DeclareLaunchArgument('diagnostic_log_file', default_value=''),
        DeclareLaunchArgument('diagnostic_log_duration', default_value='0.0'),
        DeclareLaunchArgument('diagnostic_log_filter_tag', default_value='0'),
        DeclareLaunchArgument('world', default_value='empty.sdf', description='Gazebo world file'),
        DeclareLaunchArgument('show_gazebo_gui', default_value='false', description='Show Gazebo GUI'),
        DeclareLaunchArgument('show_rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument('publish_world_to_base', default_value='true', description='Publish static world->base TF'),
        DeclareLaunchArgument('controller_debug', default_value='false', description='Enable debug logging for controller_manager (Gazebo process)'),
    ]

    arm_id = LaunchConfiguration('arm_id')
    load_gripper = LaunchConfiguration('load_gripper')
    franka_hand = LaunchConfiguration('franka_hand')
    world = LaunchConfiguration('world')
    show_gazebo_gui = LaunchConfiguration('show_gazebo_gui')
    controller_debug = LaunchConfiguration('controller_debug')
    show_rviz = LaunchConfiguration('show_rviz')
    robot_description = OpaqueFunction(
        function=_build_robot_description, args=[arm_id, load_gripper, franka_hand])
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    set_controller_debug = SetEnvironmentVariable(name='CONTROLLER_DEBUG', value=controller_debug)
    gazebo = OpaqueFunction(function=_gazebo_include, args=[world, show_gazebo_gui, controller_debug])
    runtime_nodes = OpaqueFunction(
        function=_build_runtime_nodes,
        args=[show_rviz, controller_debug],
    )

    return LaunchDescription(
        declared_args + [
            set_controller_debug,
            gazebo,
            robot_description,
            runtime_nodes,
        ]
    )
