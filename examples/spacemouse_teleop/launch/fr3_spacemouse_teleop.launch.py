"""SpaceMouse input and CartesianCommand bridge for an independently launched robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'namespace': 'fr3',
        'device_path': '',
        'operator_position_front': 'false',
        'use_sim_time': 'false',
        'base_frame': 'fr3_link0',
        'ee_frame': 'fr3_link8',
        'output_topic': 'cartesian_command',
        'linear_scale': '0.08',
        'angular_scale': '0.35',
        'deadband': '0.04',
        'publish_hz': '100.0',
        'command_timeout': '0.25',
        'max_linear_step': '0.004',
        'max_angular_step': '0.02',
        'k_lin': '250.0',
        'k_rot': '18.0',
        'damping_ratio': '1.0',
    }
    arguments = [DeclareLaunchArgument(name, default_value=value)
                 for name, value in defaults.items()]
    bridge_parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=str)
        for name in ('base_frame', 'ee_frame', 'output_topic')
    }
    bridge_parameters['input_topic'] = 'franka_controller/target_cartesian_velocity_percent'
    bridge_parameters['use_sim_time'] = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    for name in ('linear_scale', 'angular_scale', 'deadband', 'publish_hz',
                 'command_timeout', 'max_linear_step', 'max_angular_step',
                 'k_lin', 'k_rot', 'damping_ratio'):
        bridge_parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=float)

    return LaunchDescription(arguments + [
        Node(
            package='spacemouse_publisher', executable='pyspacemouse_publisher',
            name='spacemouse_publisher', namespace=LaunchConfiguration('namespace'),
            output='screen', parameters=[{
                'device_path': ParameterValue(LaunchConfiguration('device_path'), value_type=str),
                'operator_position_front': ParameterValue(
                    LaunchConfiguration('operator_position_front'), value_type=bool),
            }],
        ),
        Node(
            package='spacemouse_teleop', executable='spacemouse_to_cartesian_command.py',
            name='spacemouse_to_cartesian_command', namespace=LaunchConfiguration('namespace'),
            output='screen', parameters=[bridge_parameters],
        ),
    ])
