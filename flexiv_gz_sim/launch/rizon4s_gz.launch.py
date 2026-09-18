"""Rizon 4s effort control in Gazebo Fortress, paused until controllers activate."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    share = Path(get_package_share_directory('flexiv_gz_sim'))
    gui = LaunchConfiguration('show_gazebo_gui').perform(context) == 'true'
    model = Path(LaunchConfiguration('model_dir').perform(context)) / 'rizon4s.urdf'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': str(share / 'worlds/rizon4s.sdf') + ('' if gui else ' -s --headless-rendering')}.items())
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-world', 'rizon4s', '-name', 'rizon4s', '-file', str(model)], output='screen')
    spawners = [Node(package='controller_manager', executable='spawner',
        arguments=[name, '--controller-manager-timeout', '60'], output='screen',
        on_exit=lambda event, context: [Shutdown(reason='Controller spawner failed')] if event.returncode else [])
        for name in ['joint_state_broadcaster', 'cartesian_impedance_controller']]
    gate = Node(package='flexiv_gz_sim', executable='resume_gazebo_when_controller_ready.py',
        arguments=['--controller', 'cartesian_impedance_controller', '--world', 'rizon4s', '--timeout', '90'],
        output='screen', on_exit=lambda event, context: [Shutdown(reason='Gazebo startup failed')] if event.returncode else [])
    return [
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(model.read_text(), value_type=str), 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'], output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=spawn,
            on_exit=lambda event, context: spawners + [gate] if event.returncode == 0 else [Shutdown(reason='Robot spawn failed')])),
        spawn,
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('flexiv_gz_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('show_gazebo_gui', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('model_dir', default_value=str(share / 'model')),
        OpaqueFunction(function=setup),
    ])
