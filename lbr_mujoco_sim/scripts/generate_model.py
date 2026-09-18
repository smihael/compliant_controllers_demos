#!/usr/bin/env python3
"""Generate matching IIWA14 URDF/MJCF from lbr_description (arm without hand)."""
import argparse
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh
import xacro
from ament_index_python.packages import get_package_share_directory

HOME = [0.0, 0.5, 0.0, -1.0, 0.0, 0.7, 0.0]
JOINTS = ['lbr_A1', 'lbr_A2', 'lbr_A3', 'lbr_A4', 'lbr_A5', 'lbr_A6', 'lbr_A7']


def add_visuals(robot, root, output):
    """Bake DAE scene transforms and export each material as a visual-only OBJ."""
    assets = root.find('asset')
    mesh_dir = output / 'visual'
    mesh_dir.mkdir(exist_ok=True)
    count = 0
    for link in robot.findall('link'):
        body = root.find(f".//body[@name='{link.attrib['name']}']")
        for visual_index, visual in enumerate(link.findall('visual')):
            mesh_element = visual.find('geometry/mesh')
            if mesh_element is None or body is None:
                raise ValueError('Expected a mesh visual on a retained IIWA14 body')
            package, relative = mesh_element.attrib['filename'][len('package://'):].split('/', 1)
            path = Path(get_package_share_directory(package)) / relative
            imported = trimesh.load_scene(path, process=False)
            origin = visual.find('origin')
            xyz = '0 0 0' if origin is None else origin.get('xyz', '0 0 0')
            rpy = '0 0 0' if origin is None else origin.get('rpy', '0 0 0')
            transform = trimesh.transformations.euler_matrix(*map(float, rpy.split()), axes='sxyz')
            transform[:3, 3] = list(map(float, xyz.split()))
            scale = np.diag([*map(float, mesh_element.get('scale', '1 1 1').split()), 1.0])
            for part_index, node in enumerate(imported.graph.nodes_geometry):
                placement, geometry_name = imported.graph[node]
                mesh = imported.geometry[geometry_name].copy()
                mesh.apply_transform(transform @ scale @ placement)
                name = f"{link.attrib['name']}_visual_{visual_index}_{part_index}"
                material = getattr(mesh.visual, 'material', None)
                color = np.asarray(material.main_color if material is not None else [180, 180, 180, 255], dtype=float) / 255.0
                urdf_color = visual.find('material/color')
                if urdf_color is not None:
                    color = list(map(float, urdf_color.attrib['rgba'].split()))
                filename = mesh_dir / f'{name}.obj'
                filename.write_text(trimesh.exchange.obj.export_obj(
                    mesh, include_normals=True, include_texture=False, include_color=False))
                ET.SubElement(assets, 'mesh', name=name, file=str(filename), inertia='shell')
                ET.SubElement(assets, 'material', name=name,
                              rgba=' '.join(map(str, color)), specular='0.3', shininess='0.25')
                ET.SubElement(body, 'geom', name=name, type='mesh', mesh=name,
                              material=name, group='1', contype='0', conaffinity='0', mass='0')
                count += 1
    print(f'Converted {count} full-resolution visual mesh parts with materials')


def generate(output):
    output.mkdir(parents=True, exist_ok=True)
    description = Path(get_package_share_directory('lbr_description'))
    robot = ET.fromstring(xacro.process_file(
        str(description / 'urdf/iiwa14/iiwa14.xacro'),
        mappings={'robot_name': 'lbr', 'mode': 'mock'},
    ).toxml())
    for tag in ('ros2_control', 'gazebo', 'transmission'):
        for element in robot.findall(tag):
            robot.remove(element)
    # Import inertials and collision geometry first; add full visual meshes
    # separately so rendering detail does not alter contact or inertial data.
    ET.ElementTree(robot).write(output / 'iiwa14.urdf', encoding='unicode')
    physics = copy.deepcopy(robot)
    for link in physics.findall('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
    for mesh in physics.findall('.//mesh'):
        uri = mesh.attrib['filename']
        if uri.startswith('package://'):
            package, relative = uri[len('package://'):].split('/', 1)
            mesh.set('filename', str(Path(get_package_share_directory(package)) / relative))
    extension = ET.SubElement(physics, 'mujoco')
    ET.SubElement(extension, 'compiler', discardvisual='true', fusestatic='false',
                  balanceinertia='true', strippath='false')
    source = output / 'physics.urdf'
    ET.ElementTree(physics).write(source, encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(source))
    scene_path = output / 'scene.xml'
    mujoco.mj_saveLastXML(str(scene_path), model)
    scene = ET.parse(scene_path)
    root = scene.getroot()
    ET.SubElement(root, 'option', timestep='0.001', gravity='0 0 -9.81', integrator='implicitfast')
    world = root.find('worldbody')
    ET.SubElement(world, 'light', pos='0 -1 2', dir='0 0 -1', directional='true')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='2 2 0.1',
                  rgba='0.22 0.25 0.28 1', contype='1', conaffinity='2')
    # Robot-floor contact only: convex arm meshes are unsuitable for self-contact.
    for geom in world.findall('.//body/geom'):
        geom.set('contype', '2')
        geom.set('conaffinity', '1')
        geom.set('rgba', '0.85 0.87 0.9 0')
        geom.set('group', '3')
    # Reflected motor inertia stabilizes sampled effort control of the light
    # wrist links. Use viscous damping, without dry friction that can mask a
    # small Cartesian step. These are demo dynamics, not identified IIWA14 values.
    for joint in world.findall('.//joint'):
        joint.set('armature', '0.1')
        joint.set('damping', '1.0')
        joint.set('frictionloss', '0.0')
    actuator = ET.SubElement(root, 'actuator')
    for name in JOINTS:
        limit = robot.find(f"joint[@name='{name}']/limit")
        effort = float(limit.attrib['effort'])
        ET.SubElement(actuator, 'motor', name=name, joint=name, gear='1',
                      ctrllimited='true', ctrlrange=f'{-effort} {effort}')
    dynamics_only = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    add_visuals(robot, root, output)
    scene.write(scene_path, encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    np.testing.assert_allclose(model.body_mass, dynamics_only.body_mass, atol=1e-12)
    np.testing.assert_allclose(model.body_inertia, dynamics_only.body_inertia, atol=1e-12)
    assert model.nq == model.nv == model.nu == len(JOINTS), 'Expected a 7-axis IIWA14 arm'
    for index, name in enumerate(JOINTS):
        assert model.joint(index).name == name
        if model.jnt_limited[index]:
            lower, upper = model.jnt_range[index]
            assert lower < HOME[index] < upper, f'{name}: initial pose outside limits'
    data = mujoco.MjData(model)
    data.qpos[:] = HOME
    mujoco.mj_forward(model, data)
    assert data.ncon == 0, 'Initial pose must be free of contacts'
    source.unlink()
    print(f'Validated 7-joint IIWA14 model: {scene_path}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    generate(parser.parse_args().output.resolve())
