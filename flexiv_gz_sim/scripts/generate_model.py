#!/usr/bin/env python3
"""Generate a fixed-base, effort-controlled Rizon 4s for Gazebo Fortress."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory

HOME = [0.0, -0.69813, 0.0, 1.570796, 0.0, 0.69813, 0.0]
JOINTS = [f'joint{i}' for i in range(1, 8)]


def generate(output):
    output.mkdir(parents=True, exist_ok=True)
    description = Path(get_package_share_directory('flexiv_description'))
    robot = ET.fromstring(xacro.process_file(
        str(description / 'urdf/flexiv.urdf.xacro'),
        mappings={'robot_type': 'Rizon4s', 'load_gripper': 'false'},
    ).toxml())
    # This pinned Flexiv xacro emits origin/mass/inertia directly under link.
    # URDF requires an inertial wrapper; otherwise Pinocchio sees zero mass,
    # Gazebo drops the links, and MuJoCo infers unrelated collision-mesh inertia.
    # Repair only the XML structure, preserving all official numerical values.
    for link in robot.findall('link'):
        if link.find('mass') is not None:
            assert link.find('inertial') is None, 'Ambiguous inertial definition'
            inertial = ET.SubElement(link, 'inertial')
            for tag in ('origin', 'mass', 'inertia'):
                element = link.find(tag)
                assert element is not None, f"Missing {tag} on {link.get('name')}"
                link.remove(element)
                inertial.append(element)
        for index, collision in enumerate(link.findall('collision')):
            collision.set('name', f"{link.get('name')}_collision_{index}")
    for name in JOINTS:
        child = robot.find(f"joint[@name='{name}']/child").get('link')
        mass = robot.find(f"link[@name='{child}']/inertial/mass")
        assert mass is not None and float(mass.get('value')) > 0, f'{child}: invalid mass'
    control = ET.SubElement(robot, 'ros2_control', name='FlexivGazeboSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    for name, position in zip(JOINTS, HOME):
        physical = robot.find(f"joint[@name='{name}']")
        limit = physical.find('limit')
        assert float(limit.get('lower')) < position < float(limit.get('upper'))
        ET.SubElement(physical, 'dynamics', damping='1.0', friction='0.0')
        joint = ET.SubElement(control, 'joint', name=name)
        ET.SubElement(joint, 'command_interface', name='effort')
        state = ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(state, 'param', name='initial_value').text = str(position)
        ET.SubElement(joint, 'state_interface', name='velocity')
        ET.SubElement(joint, 'state_interface', name='effort')
    # Absolute local paths let Gazebo resolve the official OBJ/MTL visual assets.
    for mesh in robot.findall('.//mesh'):
        uri = mesh.get('filename')
        if uri.startswith('package://'):
            package, relative = uri[len('package://'):].split('/', 1)
            mesh.set('filename', str(Path(get_package_share_directory(package)) / relative))
    gazebo = ET.SubElement(robot, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', filename='gz_ros2_control-system',
                           name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    ET.SubElement(plugin, 'parameters').text = str(
        Path(get_package_share_directory('flexiv_gz_sim')) / 'config/rizon4s_controllers.yaml')
    ET.ElementTree(robot).write(output / 'rizon4s.urdf', encoding='unicode')
    assert len(robot.findall("joint[@type='revolute']")) == 7
    print('Validated 7-axis Rizon 4s Gazebo model')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    generate(parser.parse_args().output.resolve())
