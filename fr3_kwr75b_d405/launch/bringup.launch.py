"""Combined FR3 + Kunwei + Hand + front D405 holder. Default: real robot with live FT acquisition."""
import json
import math
import re
from pathlib import Path
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def included(package, filename, arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory(package))/'launch'/filename)),
        launch_arguments=arguments.items())


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    sensor_prefix = value('sensor_prefix')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', sensor_prefix):
        raise ValueError('sensor_prefix must be a nonempty relative ROS name without leading/trailing slashes')
    mode = value('mode')
    if mode not in ('view', 'real'):
        raise ValueError('mode must be view or real')

    def flag(name):
        text = value(name).lower()
        if text == 'auto':
            return mode == 'real'
        if text not in ('true', 'false'):
            raise ValueError(f'{name} must be true, false or auto')
        return text == 'true'

    start_hand, start_sensor = flag('start_hand'), flag('start_sensor')
    start_camera = flag('start_camera')
    if mode == 'view' and start_hand:
        raise ValueError('start_hand is real hardware; use mode:=real to connect to it')
    if mode == 'real' and not value('robot_ip'):
        raise ValueError('robot_ip is required for mode:=real')
    zero_sensor = value('zero_sensor').lower()
    if zero_sensor not in ('true', 'false'):
        raise ValueError('zero_sensor must be true or false')
    if zero_sensor == 'true' and not start_sensor:
        raise ValueError('zero_sensor requires start_sensor:=true')
    if float(value('zero_timeout')) <= 0:
        raise ValueError('zero_timeout must be positive')
    share = Path(get_package_share_directory('fr3_kwr75b_d405'))
    description_file = share/'urdf/fr3_kwr75b_d405.urdf.xacro'
    mappings = {name: value(name) for name in
                ('sensor_prefix', 'holder_xyz', 'holder_rpy', 'show_camera', 'show_fasteners')}
    actions = []
    if mode == 'view':
        description = xacro.process_file(str(description_file), mappings={
            **mappings, 'ros2_control': 'false'}).toxml()
        actions.extend([
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace='fr3_kwr75b_d405', name='state_publisher', output='screen',
                 parameters=[{'robot_description': ParameterValue(description, value_type=str),
                              'frame_prefix': 'fr3_kwr75b_d405_view/'}]),
            Node(package='joint_state_publisher', executable='joint_state_publisher',
                 namespace='fr3_kwr75b_d405', name='joint_state_publisher', output='screen',
                 parameters=[{'robot_description': ParameterValue(description, value_type=str),
                              'rate': 10, 'zeros.fr3_joint2': -0.7853981633974483,
                              'zeros.fr3_joint4': -2.356194490192345,
                              'zeros.fr3_joint6': 1.5707963267948966,
                              'zeros.fr3_joint7': 0.7853981633974483,
                              'zeros.fr3_finger_joint1': 0.025}]),
        ])
    else:
        arguments = {name: value(name) for name in (
            'robot_ip', 'controller_name', 'control_frame', 'compliance_frame',
            'robot_profile_file', 'friction_compensation_profile',
            'friction_compensation_enabled')}
        arguments.update(description_file=str(description_file),
                         description_mappings=json.dumps(mappings), load_gripper='false',
                         extra_joint_state_sources=json.dumps(
                             ['franka_gripper/joint_states'] if start_hand else []))
        actions.append(included('fr3_real', 'bringup.launch.py', arguments))
        if start_hand:
            actions.append(included('franka_gripper', 'gripper.launch.py', {
                'robot_ip': value('robot_ip'), 'robot_type': 'fr3',
                'namespace': 'fr3', 'use_fake_hardware': 'false'}))

    prefix = 'fr3_kwr75b_d405_view/' if mode == 'view' else ''
    camera_model = value('camera_model').strip().upper()
    if not re.fullmatch(r'[A-Z][A-Z0-9]*', camera_model):
        raise ValueError('camera_model must be a model number such as D405 or D435I')
    camera_name = value('camera_name').strip() or camera_model.lower()
    camera_namespace = value('camera_namespace').strip('/')
    camera_topic_prefix = '/' + '/'.join(filter(None, (camera_namespace, camera_name)))
    if start_camera:
        serial = value('camera_serial_no').strip()
        # Prevent numeric serial numbers being interpreted as integer parameters.
        if serial and not serial.startswith('_'):
            serial = '_' + serial
        # The upstream driver validates the entire launch context. Isolate its
        # arguments so robot options are neither warned about nor inherited.
        actions.append(GroupAction(scoped=True, forwarding=False, actions=[
            included('realsense2_camera', 'rs_launch.py', {
            'device_type': camera_model,
            'serial_no': serial or "''",
            'camera_name': camera_name,
            'camera_namespace': camera_namespace,
            'enable_color': 'true',
            'base_frame_id': 'link',
            'publish_tf': 'true',
        })]))
        camera_xyz = value('camera_mount_xyz').split()
        camera_rpy = value('camera_mount_rpy').split()
        if len(camera_xyz) != 3 or len(camera_rpy) != 3:
            raise ValueError('camera_mount_xyz and camera_mount_rpy each require three numbers')
        for coordinate in camera_xyz + camera_rpy:
            if not math.isfinite(float(coordinate)):
                raise ValueError('camera mount coordinates must be finite')
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=camera_name + '_mount_tf', output='screen',
            arguments=[
                '--x', camera_xyz[0], '--y', camera_xyz[1], '--z', camera_xyz[2],
                '--roll', camera_rpy[0], '--pitch', camera_rpy[1], '--yaw', camera_rpy[2],
                '--frame-id', prefix + 'd405_holder',
                '--child-frame-id', camera_name + '_link',
            ]))
    if start_sensor:
        actions.append(included('kunwei_ft_driver', 'publish.launch.py', {
            'host': value('sensor_host'), 'port': value('sensor_port'),
            'sensor_prefix': sensor_prefix, 'sensor_frame': prefix+sensor_prefix,
            'publish_visualization': value('rviz'),
            'visualization_rate': value('visualization_rate'),
        }))
        actions.append(Node(
            package='fr3_kwr75b_d405', executable='republish_tcp_wrench.py',
            output='screen', parameters=[{
                'input_topic': '/' + sensor_prefix + '/wrench',
                'output_topic': '/' + sensor_prefix + '/wrench_tcp',
                'target_frame': prefix + 'fr3_hand_tcp',
            }]))
    if zero_sensor == 'true':
        actions.append(Node(
            package='fr3_kwr75b_d405', executable='zero_ft_sensor.py',
            output='screen', parameters=[{
                'sensor_prefix': sensor_prefix,
                'timeout': float(value('zero_timeout')),
            }]))
    if value('rviz').lower() == 'true':
        with open(share/'config/fr3_kwr75b_d405.rviz', encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        vm = config['Visualization Manager']
        vm['Global Options']['Fixed Frame'] = prefix+'fr3_link0'
        for display in vm['Displays']:
            if display['Class'] == 'rviz_default_plugins/RobotModel':
                display['TF Prefix'] = prefix.rstrip('/')
                display['Description Topic']['Value'] = (
                    '/fr3_kwr75b_d405/robot_description' if mode == 'view' else '/fr3/robot_description')
            if display['Class'] == 'rviz_default_plugins/Axes':
                frame = display['Reference Frame']
                if frame.startswith('ft_sensor'):
                    frame = sensor_prefix + frame[len('ft_sensor'):]
                display['Reference Frame'] = prefix+frame
            if display['Class'] == 'rviz_default_plugins/Wrench':
                display['Topic']['Value'] = '/'+sensor_prefix+'/wrench_visualization'
            if display['Class'] == 'rviz_default_plugins/Image':
                display['Topic']['Value'] = camera_topic_prefix + '/color/image_raw'
        vm['Views']['Current']['Target Frame'] = prefix+'fr3_link8'
        runtime = tempfile.TemporaryDirectory(prefix='fr3-kwr75b-rviz-')
        path = Path(runtime.name)/'fr3_kwr75b_d405.rviz'
        path.write_text(yaml.safe_dump(config, sort_keys=False))

        def cleanup(_context):
            runtime.cleanup()
            return []

        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
        actions.append(Node(package='rviz2', executable='rviz2', name='fr3_kwr75b_d405_rviz',
                            arguments=['-d', str(path)], output='screen'))
    return actions


def generate_launch_description():
    defaults = [
        ('mode', 'real', 'real starts the shared fr3_real bringup; view previews the assembly'),
        ('robot_ip', '192.168.1.1', 'FCI IP or hostname'),
        ('sensor_prefix', 'ft_sensor', 'Relative ROS prefix for sensor topics, zero service and frames'),
        ('sensor_host', '192.168.1.88', 'Kunwei TCP address'),
        ('sensor_port', '5152', 'Kunwei TCP port'),
        ('start_sensor', 'true', 'Start FT acquisition; auto enables it only in real mode'),
        ('zero_sensor', 'false', 'Set software FT zero once fresh samples arrive at startup'),
        ('zero_timeout', '30.0', 'Seconds to wait for startup zeroing'),
        ('start_hand', 'auto', 'auto: true only in real mode'),
        ('start_camera', 'true', 'Start the RealSense driver; auto enables it only in real mode'),
        ('camera_model', 'D405', 'RealSense model number used for device_type selection, e.g. D405 or D435I'),
        ('camera_serial_no', '', 'Optional RealSense serial number to select among cameras of the same model'),
        ('camera_mount_xyz', '0.0149540784861047 0 0.0410267986812785', 'Camera base origin in d405_holder, metres; CAD preview until calibrated'),
        ('camera_mount_rpy', '0 0.4363323129985824 0', 'Camera base orientation in d405_holder, radians; CAD preview until calibrated'),
        ('camera_name', '', 'RealSense node name and frame prefix; empty uses lowercase camera_model'),
        ('camera_namespace', '', 'Optional RealSense ROS namespace'),
        ('visualization_rate', '30.0', 'Wrench display output cap; full-rate output unchanged'),
        ('holder_xyz', '0.02045 0 0.0175', 'Holder source origin in Hand frame, metres'),
        ('holder_rpy', '3.141592653589793 -1.5707963267948966 0', 'Holder CAD axes to Hand axes, radians'),
        ('show_camera', 'true', 'Show source camera envelope and uncalibrated preview optical frame'),
        ('show_fasteners', 'false', 'Show attachment screw paths, including two camera M3 screws'),
        ('rviz', 'true', 'Start RViz'),
        ('controller_name', 'cartesian_impedance_controller', 'Passed to fr3_real in real mode'),
        ('control_frame', 'fr3_link8', 'Controller frame; e.g.fr3_hand_tcp if desired'),
        ('compliance_frame', '', 'Passed to fr3_real'),
        ('robot_profile_file', '', 'Passed to fr3_real'),
        ('friction_compensation_profile', 'friction_compensation_sigmoid', 'Passed to fr3_real'),
        ('friction_compensation_enabled', 'false', 'Passed to fr3_real'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in defaults
    ]+[OpaqueFunction(function=setup)])
