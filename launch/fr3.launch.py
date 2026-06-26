# Real FR3/Panda robot launch with selectable Cartesian or joint compliant controller.

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
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


def is_joint_controller(controller_name):
    return controller_name == 'joint_impedance_controller'


def controller_include(context):
    controller_name = LaunchConfiguration('controller_name').perform(context)
    impl_library = LaunchConfiguration('impl_library').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    robot_state_topic = f'/{namespace}/franka_robot_state_broadcaster/robot_state' if namespace else '/franka_robot_state_broadcaster/robot_state'
    if is_joint_controller(controller_name):
        launch_file = 'joint_wrapper.launch.py'
        launch_arguments = {
            'namespace': LaunchConfiguration('namespace'),
            'arm_id': LaunchConfiguration('arm_id'),
            'controller_name': LaunchConfiguration('controller_name'),
            'ee_frame': LaunchConfiguration('ee_frame'),
            'robot_description_node': 'robot_state_publisher',
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
        launch_file = 'cartesian_wrapper.launch.py'
        launch_arguments = {
            'robot_profile': LaunchConfiguration('robot_profile'),
            'namespace': LaunchConfiguration('namespace'),
            'arm_id': LaunchConfiguration('arm_id'),
            'controller_name': LaunchConfiguration('controller_name'),
            'init_k_pos': LaunchConfiguration('init_k_pos'),
            'init_k_ori': LaunchConfiguration('init_k_ori'),
            'ee_frame': LaunchConfiguration('ee_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'robot_description_node': 'robot_state_publisher',
            'robot_description_param': 'robot_description',
            'end_effector_profile_node': 'end_effector_profile_server',
            'end_effector_robot_state_topic': robot_state_topic,
            'tcp_enabled': LaunchConfiguration('tcp_enabled'),
            'gravity_compensation_enabled': LaunchConfiguration('gravity_compensation_enabled'),
            'ee_load_compensation_enabled': LaunchConfiguration('ee_load_compensation_enabled'),
            'friction_compensation_enabled': LaunchConfiguration('friction_compensation_enabled'),
            'friction_model': LaunchConfiguration('friction_model'),
            'friction_scale': LaunchConfiguration('friction_scale'),
            'friction_use_gating': LaunchConfiguration('friction_use_gating'),
            'plugin_params_file': LaunchConfiguration('plugin_params_file'),
            'csv_file': LaunchConfiguration('csv_file'),
            'diagnostic_log_file': LaunchConfiguration('diagnostic_log_file'),
            'diagnostic_log_duration': LaunchConfiguration('diagnostic_log_duration'),
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'shutdown_on_done': LaunchConfiguration('shutdown_on_done'),
            'publish_world_to_base': LaunchConfiguration('publish_world_to_base'),
            'load_end_effector_profile': LaunchConfiguration('load_end_effector_profile'),
            'end_effector_profile': LaunchConfiguration('end_effector_profile'),
        }
    launch_arguments['impl_library'] = impl_library or (
        'libjoint_impedance_impl.so' if is_joint_controller(controller_name)
        else 'libcartesian_impedance_impl.so'
    )

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('compliant_controllers'),
                'launch',
                launch_file,
            ])
        ),
        launch_arguments=launch_arguments.items(),
    )]


def generate_launch_description():
    robot_profile = os.environ.get('COMPLIANT_ROBOT_PROFILE', 'ROBOT_1')
    robot_cfg = load_robot_profile({
        'robot_name': 'fr3',
        'robot_ip': '192.168.1.1',
    }, robot_profile)

    default_end_effector_profile = os.path.join(
        get_package_share_directory('compliant_controllers_demos'),
        'config',
        'franka_hand_default.endeffector-profile.json',
    )

    declared_args = [
        DeclareLaunchArgument('robot_profile', default_value=robot_profile),
        DeclareLaunchArgument('robot_type', default_value=str(robot_cfg.get('robot_type', robot_cfg.get('robot_name', 'fr3')))),
        DeclareLaunchArgument('arm_id', default_value=str(robot_cfg.get('robot_name', 'fr3'))),
        DeclareLaunchArgument('arm_prefix', default_value=''),
        DeclareLaunchArgument('namespace', default_value='fr3'),
        DeclareLaunchArgument('robot_ip', default_value=str(robot_cfg.get('robot_ip', '192.168.1.1'))),
        DeclareLaunchArgument('load_gripper', default_value='false'),
        DeclareLaunchArgument('joint_state_rate', default_value='30'),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller'),
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
        DeclareLaunchArgument('tcp_enabled', default_value='true'),
        DeclareLaunchArgument('gravity_compensation_enabled', default_value='false'),
        DeclareLaunchArgument('ee_load_compensation_enabled', default_value='false'),
        DeclareLaunchArgument('friction_compensation_enabled', default_value='false'),
        DeclareLaunchArgument('friction_model', default_value='auto'),
        DeclareLaunchArgument('friction_scale', default_value='1.0'),
        DeclareLaunchArgument('friction_use_gating', default_value='true'),
        DeclareLaunchArgument('plugin_params_file', default_value=''),
        DeclareLaunchArgument('csv_file', default_value=''),
        DeclareLaunchArgument('diagnostic_log_file', default_value=''),
        DeclareLaunchArgument('diagnostic_log_duration', default_value='0.0'),
        DeclareLaunchArgument('diagnostic_mode', default_value='0'),
        DeclareLaunchArgument('shutdown_on_done', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('publish_world_to_base', default_value='true'),
        DeclareLaunchArgument('load_end_effector_profile', default_value='true'),
        DeclareLaunchArgument('end_effector_profile', default_value=default_end_effector_profile),
    ]

    include_franka = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'])
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
                FindPackageShare('compliant_controllers_demos'), 'config', 'fr3_controllers.yaml'
            ]),
        }.items(),
    )

    include_controller = OpaqueFunction(function=controller_include)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=LaunchConfiguration('namespace'),
        arguments=['--display-config', PathJoinSubstitution([
            FindPackageShare('franka_description'), 'rviz', 'visualize_franka.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription(declared_args + [include_franka, include_controller, rviz_node])
