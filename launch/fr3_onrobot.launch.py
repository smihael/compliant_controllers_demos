import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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


def is_true(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def is_joint_controller(controller_name):
    return controller_name == 'joint_impedance_controller'


def robot_description_command(robot_model):
    return [
        'xacro ',
        robot_model,
        ' robot_type:=', LaunchConfiguration('robot_type'),
        ' arm_prefix:=', LaunchConfiguration('arm_prefix'),
        ' no_prefix:=', LaunchConfiguration('no_prefix'),
        ' ros2_control:=', LaunchConfiguration('launch_fr3_control'),
        ' robot_ip:=', LaunchConfiguration('robot_ip'),
        ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware'),
        ' fake_sensor_commands:=', LaunchConfiguration('fake_sensor_commands'),
        ' ft_prefix:=', LaunchConfiguration('ft_prefix'),
        ' ft_parent:=', LaunchConfiguration('ft_parent'),
        ' xyz_onrobot:="', LaunchConfiguration('xyz_onrobot'), '"',
        ' rpy_onrobot:="', LaunchConfiguration('rpy_onrobot'), '"',
        ' onrobot_ip_address:=', LaunchConfiguration('onrobot_ip_address'),
        ' onrobot_sampling_rate:=', LaunchConfiguration('onrobot_sampling_rate'),
        ' onrobot_internal_filter_rate:=', LaunchConfiguration('onrobot_internal_filter_rate'),
        ' onrobot_use_hardware_biasing:=', LaunchConfiguration('onrobot_use_hardware_biasing'),
    ]


def robot_description_parameter(robot_model):
    return {
        'robot_description': ParameterValue(Command(robot_description_command(robot_model)), value_type=str),
    }


def controller_include(context):
    if not is_true(LaunchConfiguration('launch_fr3_control').perform(context)):
        return []

    controller_name = LaunchConfiguration('controller_name').perform(context)
    impl_library = LaunchConfiguration('impl_library').perform(context)
    robot_description_node = 'robot_state_publisher'
    if is_joint_controller(controller_name):
        launch_file = 'joint_wrapper.launch.py'
        launch_arguments = {
            'namespace': LaunchConfiguration('namespace'),
            'arm_id': LaunchConfiguration('arm_id'),
            'controller_name': LaunchConfiguration('controller_name'),
            'ee_frame': LaunchConfiguration('ee_frame'),
            'robot_description_node': robot_description_node,
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
            'robot_description_node': robot_description_node,
            'robot_description_param': 'robot_description',
            'end_effector_profile_node': LaunchConfiguration('end_effector_profile_node'),
            'tcp_enabled': LaunchConfiguration('tcp_enabled'),
            'tcp_x': LaunchConfiguration('tcp_x'),
            'tcp_y': LaunchConfiguration('tcp_y'),
            'tcp_z': LaunchConfiguration('tcp_z'),
            'tcp_roll': LaunchConfiguration('tcp_roll'),
            'tcp_pitch': LaunchConfiguration('tcp_pitch'),
            'tcp_yaw': LaunchConfiguration('tcp_yaw'),
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
            'diagnostic_log_filter_tag': LaunchConfiguration('diagnostic_log_filter_tag'),
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


def fr3_control_nodes(context, robot_model):
    if not is_true(LaunchConfiguration('launch_fr3_control').perform(context)):
        return []

    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    robot_description_topic = f'/{namespace}/robot_description' if namespace else '/robot_description'
    broadcaster_key = f'/{namespace}/franka_robot_state_broadcaster' if namespace else '/franka_robot_state_broadcaster'
    broadcaster_params_path = os.path.join(
        tempfile.gettempdir(),
        f'fr3_onrobot_franka_robot_state_broadcaster_{namespace or "root"}.yaml',
    )
    robot_description = Command(robot_description_command(robot_model)).perform(context)
    with open(broadcaster_params_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump({
            broadcaster_key: {
                'ros__parameters': {
                    'robot_description': robot_description,
                },
            },
        }, f, sort_keys=False)

    joint_state_publisher_sources = [
        'franka/joint_states',
        'franka_gripper/joint_states',
    ]

    return [
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('compliant_controllers_demos'),
                    'config',
                    'fr3_controllers.yaml',
                ]),
                {'robot_type': LaunchConfiguration('robot_type')},
                {'load_gripper': False},
                {'arm_prefix': LaunchConfiguration('arm_prefix')},
            ],
            remappings=[
                ('joint_states', joint_state_publisher_sources[0]),
                ('~/robot_description', robot_description_topic),
            ],
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'source_list': joint_state_publisher_sources,
                'rate': ParameterValue(LaunchConfiguration('joint_state_rate'), value_type=int),
                'use_robot_description': False,
            }],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('namespace'),
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('namespace'),
            arguments=[
                'franka_robot_state_broadcaster',
                '--param-file',
                broadcaster_params_path,
            ],
            condition=UnlessCondition(LaunchConfiguration('use_fake_hardware')),
            output='screen',
        ),
    ]


def generate_launch_description():
    robot_profile = os.environ.get('COMPLIANT_ROBOT_PROFILE', 'ROBOT_1')
    robot_cfg = load_robot_profile({
        'robot_name': 'fr3',
        'robot_ip': '192.168.1.1',
    }, robot_profile)

    robot_model = PathJoinSubstitution([
        FindPackageShare('compliant_controllers_demos'),
        'urdf',
        'fr3_onrobot_ft.urdf.xacro',
    ])

    declared_args = [
        DeclareLaunchArgument('robot_profile', default_value=robot_profile),
        DeclareLaunchArgument('robot_type', default_value='fr3'),
        DeclareLaunchArgument('arm_id', default_value=str(robot_cfg.get('robot_name', 'fr3'))),
        DeclareLaunchArgument('arm_prefix', default_value=''),
        DeclareLaunchArgument('namespace', default_value='fr3'),
        DeclareLaunchArgument('robot_ip', default_value=str(robot_cfg.get('robot_ip', '192.168.1.1'))),
        DeclareLaunchArgument('launch_fr3_control', default_value='false'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false'),
        DeclareLaunchArgument('no_prefix', default_value='false'),
        DeclareLaunchArgument('ft_prefix', default_value='onrobot_'),
        DeclareLaunchArgument('ft_parent', default_value=''),
        DeclareLaunchArgument('xyz_onrobot', default_value='0 0 0'),
        DeclareLaunchArgument('rpy_onrobot', default_value='0 0 -1.5707963267948966'),
        DeclareLaunchArgument('onrobot_ip_address', default_value='192.168.1.1'),
        DeclareLaunchArgument('onrobot_sensor_id', default_value='onrobot_ft'),
        DeclareLaunchArgument('onrobot_topic_name', default_value='wrench'),
        DeclareLaunchArgument('onrobot_port', default_value='49152'),
        DeclareLaunchArgument('onrobot_samples_per_request', default_value='10'),
        DeclareLaunchArgument('onrobot_speed', default_value='10'),
        DeclareLaunchArgument('onrobot_filter', default_value='4'),
        DeclareLaunchArgument('onrobot_bias_on_start', default_value='false'),
        DeclareLaunchArgument('onrobot_sampling_rate', default_value='500'),
        DeclareLaunchArgument('onrobot_internal_filter_rate', default_value='0'),
        DeclareLaunchArgument('onrobot_use_hardware_biasing', default_value='false'),
        DeclareLaunchArgument('joint_state_rate', default_value='30'),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller'),
        DeclareLaunchArgument('impl_library', default_value='libcartesian_impedance_impl.so'),
        DeclareLaunchArgument('init_k_pos', default_value='200.0'),
        DeclareLaunchArgument('init_k_ori', default_value='10.0'),
        DeclareLaunchArgument('joint_initial_stiffness', default_value='600,600,600,600,250,150,50'),
        DeclareLaunchArgument('joint_initial_damping', default_value='30,30,30,30,10,10,5'),
        DeclareLaunchArgument('joint_filter_alpha', default_value='0.99'),
        DeclareLaunchArgument('joint_max_tau_delta', default_value='1.0'),
        DeclareLaunchArgument('joint_power_enable_tau_norm_threshold', default_value='1.1'),
        DeclareLaunchArgument('joint_max_power_enable_count', default_value='100'),
        DeclareLaunchArgument('ee_frame', default_value=''),
        DeclareLaunchArgument('base_frame', default_value='base'),
        DeclareLaunchArgument('tcp_enabled', default_value='true'),
        DeclareLaunchArgument('tcp_x', default_value='0.0'),
        DeclareLaunchArgument('tcp_y', default_value='0.0'),
        DeclareLaunchArgument('tcp_z', default_value='0.195'),
        DeclareLaunchArgument('tcp_roll', default_value='0.0'),
        DeclareLaunchArgument('tcp_pitch', default_value='0.0'),
        DeclareLaunchArgument('tcp_yaw', default_value='0.0'),
        DeclareLaunchArgument('end_effector_profile_node', default_value=''),
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
        DeclareLaunchArgument('diagnostic_log_filter_tag', default_value='0'),
        DeclareLaunchArgument('shutdown_on_done', default_value='false'),
        DeclareLaunchArgument('publish_world_to_base', default_value='true'),
        DeclareLaunchArgument('load_end_effector_profile', default_value='false'),
        DeclareLaunchArgument('end_effector_profile', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([
            FindPackageShare('compliant_controllers_demos'),
            'config',
            'fr3_onrobot.rviz',
        ])),
        DeclareLaunchArgument('fixed_frame', default_value='fr3_link0'),
    ]

    robot_description = robot_description_parameter(robot_model)

    description_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[robot_description],
        output='screen',
    )

    control_nodes = OpaqueFunction(function=fr3_control_nodes, args=[robot_model])
    include_controller = OpaqueFunction(function=controller_include)

    onrobot_node = Node(
        package='onrobot_ft_ros2',
        executable='onrobot_ft_udp_node',
        name='onrobot_ft_udp_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'ip_address': LaunchConfiguration('onrobot_ip_address'),
            'sensor_id': LaunchConfiguration('onrobot_sensor_id'),
            'topic_name': LaunchConfiguration('onrobot_topic_name'),
            'port': ParameterValue(LaunchConfiguration('onrobot_port'), value_type=int),
            'samples_per_request': ParameterValue(
                LaunchConfiguration('onrobot_samples_per_request'), value_type=int),
            'speed': ParameterValue(LaunchConfiguration('onrobot_speed'), value_type=int),
            'filter': ParameterValue(LaunchConfiguration('onrobot_filter'), value_type=int),
            'bias_on_start': ParameterValue(
                LaunchConfiguration('onrobot_bias_on_start'), value_type=bool),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            LaunchConfiguration('rviz_config'),
            '-f',
            LaunchConfiguration('fixed_frame'),
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription(declared_args + [
        description_publisher,
        control_nodes,
        include_controller,
        onrobot_node,
        rviz,
    ])
