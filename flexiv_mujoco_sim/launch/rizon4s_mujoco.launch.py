"""Rizon 4s arm using the ros-controls MuJoCo hardware plugin."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HOME = [0.0, -0.69813, 0.0, 1.570796, 0.0, 0.69813, 0.0]
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']


def setup(context):
    share = Path(get_package_share_directory('flexiv_mujoco_sim'))
    model_dir = Path(LaunchConfiguration('model_dir').perform(context))
    robot = ET.parse(model_dir / 'rizon4s.urdf').getroot()
    control = ET.SubElement(robot, 'ros2_control', name='FlexivMujocoSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'mujoco_ros2_control/MujocoSystemInterface'
    for name, value in {
        'mujoco_model': str(model_dir / 'scene.xml'),
        'headless': LaunchConfiguration('headless').perform(context),
        'sim_speed_factor': '1.0',
    }.items():
        ET.SubElement(hardware, 'param', name=name).text = value
    for name, position in zip(JOINTS, HOME):
        joint = ET.SubElement(control, 'joint', name=name)
        ET.SubElement(joint, 'command_interface', name='effort')
        state = ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(state, 'param', name='initial_value').text = str(position)
        ET.SubElement(joint, 'state_interface', name='velocity')
        ET.SubElement(joint, 'state_interface', name='effort')
    description = ParameterValue(ET.tostring(robot, encoding='unicode'), value_type=str)
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', parameters=[{'robot_description': description}],
             output='screen'),
        # Wall time allows controller activation while MuJoCo is paused. Physics
        # runs at 1x real time; /clock remains available for external clients.
        Node(package='mujoco_ros2_control', executable='ros2_control_node',
             parameters=[str(share / 'config/rizon4s_controllers.yaml'),
                 {'use_sim_time': False, 'robot_description': description}],
             remappings=[('~/robot_description', '/robot_description')],
             output='screen', on_exit=Shutdown()),
        Node(package='flexiv_mujoco_sim', executable='start_controllers.py',
             arguments=['--controller', LaunchConfiguration('controller_name')],
             output='screen', on_exit=lambda event, context: (
                 [Shutdown(reason='Controller startup failed; see startup log')]
                 if event.returncode else [])),
    ]


def generate_launch_description():
    share = get_package_share_directory('flexiv_mujoco_sim')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller',
            choices=['cartesian_impedance_controller']),
        DeclareLaunchArgument('model_dir', default_value=str(Path(share) / 'model')),
        OpaqueFunction(function=setup),
    ])
