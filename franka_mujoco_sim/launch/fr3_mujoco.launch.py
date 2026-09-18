"""FR3 arm using the ros-controls MuJoCo hardware plugin."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HOME = [0.0, -0.7853981634, 0.0, -2.3561944902, 0.0, 1.5707963268, 0.7853981634]


def setup(context):
    share = Path(get_package_share_directory('franka_mujoco_sim'))
    model_dir = Path(LaunchConfiguration('model_dir').perform(context))
    robot = ET.parse(model_dir / 'fr3.urdf').getroot()
    control = ET.SubElement(robot, 'ros2_control', name='Fr3MujocoSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'mujoco_ros2_control/MujocoSystemInterface'
    for name, value in {
        'mujoco_model': str(model_dir / 'scene.xml'),
        'headless': LaunchConfiguration('headless').perform(context),
        'sim_speed_factor': '1.0',
    }.items():
        ET.SubElement(hardware, 'param', name=name).text = value
    for index, position in enumerate(HOME, 1):
        joint = ET.SubElement(control, 'joint', name=f'fr3_joint{index}')
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
             parameters=[str(share / 'config/fr3_controllers.yaml'),
                 {'use_sim_time': False, 'robot_description': description}],
             remappings=[('~/robot_description', '/robot_description')],
             output='screen', on_exit=Shutdown()),
        Node(package='franka_mujoco_sim', executable='start_controllers.py',
             arguments=['--controller', LaunchConfiguration('controller_name')],
             output='screen', on_exit=lambda event, context: (
                 [Shutdown(reason='Controller startup failed; see startup log')]
                 if event.returncode else [])),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'world', 'base']),
    ]


def generate_launch_description():
    share = get_package_share_directory('franka_mujoco_sim')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('controller_name', default_value='cartesian_impedance_controller',
            choices=['cartesian_impedance_controller', 'joint_impedance_controller', 'fr3_arm_controller']),
        DeclareLaunchArgument('model_dir', default_value=str(Path(share) / 'model')),
        OpaqueFunction(function=setup),
    ])
