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


def _build_robot_description(
        context: LaunchContext, arm_id, load_gripper, franka_hand,
        extra_urdf_file):
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

    # Small Cartesian commands otherwise stall against upstream dry friction.
    # Keep this simulator setting separate from controller friction compensation.
    joint_friction = context.perform_substitution(LaunchConfiguration('joint_friction'))
    for joint in xacro_doc.getElementsByTagName('joint'):
        if joint.getAttribute('name') in {f'{arm_id_str}_joint{i}' for i in range(1, 8)}:
            for dynamics in joint.getElementsByTagName('dynamics'):
                dynamics.setAttribute('friction', joint_friction)

    default_yaml = os.path.join(
        get_package_share_directory('franka_gazebo_bringup'),
        'config', 'franka_gazebo_controllers.yaml'
    )
    custom_yaml = os.path.join(
        get_package_share_directory('franka_gz_sim'),
        'config', 'fr3_gz_controllers.yaml'
    )
    urdf_xml = xacro_doc.toxml().replace(default_yaml, custom_yaml)
    fragment_path = context.perform_substitution(extra_urdf_file)
    if fragment_path:
        with open(fragment_path, encoding='utf-8') as fragment_file:
            fragment_xml = fragment_file.read().strip()
        urdf_xml = urdf_xml.replace('</robot>', f'{fragment_xml}\n</robot>')
    return [Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
        output='both', parameters=[{'robot_description': urdf_xml}]
    )]


def _gazebo_include(context: LaunchContext, world, show_gazebo_gui, controller_debug):
    world_file = context.perform_substitution(world)
    gui_flag = context.perform_substitution(show_gazebo_gui).lower() in ('true', '1', 'yes')
    debug_flag = context.perform_substitution(controller_debug).lower() in ('true', '1', 'yes')

    if gui_flag:
        gz_args = world_file
    else:
        verbosity = '4' if debug_flag else '2'
        gz_args = f"{world_file} -v {verbosity} -s --headless-rendering"

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    return [gazebo_launch]


def _is_joint_controller(controller_name):
    return controller_name == 'joint_impedance_controller'


def _build_runtime_nodes(
        context: LaunchContext, show_rviz, controller_debug, world,
        startup_timeout):
    namespace = context.perform_substitution(LaunchConfiguration('namespace'))
    arm_id = context.perform_substitution(LaunchConfiguration('arm_id'))
    controller_manager = f'/{namespace}/controller_manager' if namespace else '/controller_manager'
    rviz_flag = context.perform_substitution(show_rviz).lower() in ('true', '1', 'yes')
    controller_name = context.perform_substitution(LaunchConfiguration('controller_name'))
    world_name = os.path.splitext(os.path.basename(context.perform_substitution(world)))[0]
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
        }
    else:
        controller_launch_file = 'cartesian_wrapper.launch.py'
        controller_launch_arguments = {
            'namespace': LaunchConfiguration('namespace'),
            'arm_id': LaunchConfiguration('arm_id'),
            'controller_name': LaunchConfiguration('controller_name'),
            'controller_manager': controller_manager,
            'ee_frame': LaunchConfiguration('ee_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'robot_description_node': '/robot_state_publisher',
            'robot_description_param': 'robot_description',
            'dithering_enabled': LaunchConfiguration('dithering_enabled'),
            'gravity_compensation_enabled': LaunchConfiguration(
                'gravity_compensation_enabled'),
            'friction_compensation_enabled': LaunchConfiguration('friction_compensation_enabled'),
            'max_step_guard_enabled': LaunchConfiguration('max_step_guard_enabled'),
            'friction_model': LaunchConfiguration('friction_model'),
            'friction_scale': LaunchConfiguration('friction_scale'),
            'friction_use_gating': LaunchConfiguration('friction_use_gating'),
            'log_file': LaunchConfiguration('log_file'),
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

    resume_world = Node(
        package='franka_gz_sim',
        executable='resume_gazebo_when_controller_ready.py',
        name='resume_gazebo_when_controller_ready',
        output='screen',
        arguments=[
            '--controller-manager', controller_manager,
            '--controller', controller_name,
            '--world', world_name,
            '--timeout', startup_timeout,
        ],
    )

    nodes = [spawn, spawn_to_controllers, resume_world]

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
        DeclareLaunchArgument('joint_friction', default_value='0.0',
                              description='Simulated arm joint dry friction in Nm; upstream uses 0.2'),
        DeclareLaunchArgument('load_gripper', default_value='false', description='Load gripper in URDF'),
        DeclareLaunchArgument(
            'extra_urdf_file', default_value='',
            description='Optional URDF fragment appended to the robot description'),
        DeclareLaunchArgument('franka_hand', default_value='franka_hand', description='Gripper variant'),
        DeclareLaunchArgument(
            'startup_timeout', default_value='30.0',
            description='Maximum seconds to wait for controller initialization while Gazebo is paused'),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller', description='Primary controller to spawn'),
        DeclareLaunchArgument('impl_library', default_value=''),
        DeclareLaunchArgument('ee_frame', default_value=''),
        DeclareLaunchArgument('base_frame', default_value=''),
        DeclareLaunchArgument('dithering_enabled', default_value='false'),
        DeclareLaunchArgument('gravity_compensation_enabled', default_value=''),
        DeclareLaunchArgument('friction_compensation_enabled', default_value='false'),
        DeclareLaunchArgument('max_step_guard_enabled', default_value=''),
        DeclareLaunchArgument('friction_model', default_value='auto'),
        DeclareLaunchArgument('friction_scale', default_value='1.0'),
        DeclareLaunchArgument('friction_use_gating', default_value='true'),
        DeclareLaunchArgument('log_file', default_value=''),
        DeclareLaunchArgument('world', default_value='empty.sdf', description='Gazebo world file'),
        DeclareLaunchArgument('show_gazebo_gui', default_value='false', description='Show Gazebo GUI'),
        DeclareLaunchArgument('show_rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument('publish_world_to_base', default_value='true', description='Publish static world->base TF'),
        DeclareLaunchArgument('controller_debug', default_value='false', description='Enable debug logging for controller_manager (Gazebo process)'),
    ]

    arm_id = LaunchConfiguration('arm_id')
    load_gripper = LaunchConfiguration('load_gripper')
    extra_urdf_file = LaunchConfiguration('extra_urdf_file')
    franka_hand = LaunchConfiguration('franka_hand')
    startup_timeout = LaunchConfiguration('startup_timeout')
    world = LaunchConfiguration('world')
    show_gazebo_gui = LaunchConfiguration('show_gazebo_gui')
    controller_debug = LaunchConfiguration('controller_debug')
    show_rviz = LaunchConfiguration('show_rviz')
    robot_description = OpaqueFunction(
        function=_build_robot_description,
        args=[arm_id, load_gripper, franka_hand, extra_urdf_file])
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    set_controller_debug = SetEnvironmentVariable(name='CONTROLLER_DEBUG', value=controller_debug)
    gazebo = OpaqueFunction(
        function=_gazebo_include, args=[world, show_gazebo_gui, controller_debug])
    runtime_nodes = OpaqueFunction(
        function=_build_runtime_nodes,
        args=[show_rviz, controller_debug, world, startup_timeout],
    )

    return LaunchDescription(
        declared_args + [
            set_controller_debug,
            gazebo,
            robot_description,
            runtime_nodes,
        ]
    )
