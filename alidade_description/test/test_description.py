"""Verifie que la description se genere pour chaque cible et reste coherente.

Lance par colcon test. Necessite xacro et check_urdf (liburdfdom-tools).
"""
import math
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

PKG = Path(__file__).resolve().parent.parent
REVISION = yaml.safe_load((PKG / 'config' / 'alidade_v1.yaml').read_text())


def share_dir() -> Path:
    """Le xacro resout $(find alidade_description) par ament_index : le test
    s'execute sur le paquet installe, comme le fera le robot."""
    try:
        return Path(get_package_share_directory('alidade_description'))
    except PackageNotFoundError:
        pytest.skip('alidade_description non installe : lancer via colcon test')


def run_xacro(**args) -> ET.Element:
    cmd = ['xacro', str(share_dir() / 'urdf' / 'alidade.urdf.xacro')]
    cmd += [f'{k}:={v}' for k, v in args.items()]
    out = subprocess.run(cmd, capture_output=True, text=True, check=True)
    return ET.fromstring(out.stdout)


@pytest.mark.parametrize('target,mock,plugin', [
    ('real', 'false', 'alidade_hardware/PicoSystem'),
    ('real', 'true', 'mock_components/GenericSystem'),
    ('sim', 'false', 'gz_ros2_control/GazeboSimSystem'),
])
def test_targets(target, mock, plugin):
    robot = run_xacro(target=target, use_mock_hardware=mock)
    assert robot.find('ros2_control/hardware/plugin').text == plugin
    joints = {j.get('name') for j in robot.findall('ros2_control/joint')}
    assert joints == {'wheel_left_joint', 'wheel_right_joint'}
    gazebo = robot.findall('gazebo')
    assert bool(gazebo) == (target == 'sim')


def test_constants_come_from_revision():
    robot = run_xacro(target='real')
    params = {p.get('name'): p.text for p in robot.findall('ros2_control/hardware/param')}
    assert int(params['counts_per_rev']) == REVISION['kinematics']['counts_per_rev']


def test_frames_and_root():
    robot = run_xacro(target='real')
    links = {l.get('name') for l in robot.findall('link')}
    assert 'base_footprint' in links and 'root' not in links
    joints = {j.get('name') for j in robot.findall('joint')}
    assert not (links & joints), 'Gazebo refuse les noms partages link/joint'


def rotation(rpy):
    """Matrice d'une origine URDF : rotations fixes x, puis y, puis z."""
    (cr, cp, cy), (sr, sp, sy) = [[f(a) for a in rpy] for f in (math.cos, math.sin)]
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def orientation(robot, frame):
    """Orientation de frame dans base_footprint, en remontant les joints."""
    parents = {j.find('child').get('link'): j for j in robot.findall('joint')}
    rot = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    while frame in parents:
        joint = parents[frame]
        origin = joint.find('origin')
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        joint_rot = rotation(rpy)
        rot = [[sum(joint_rot[i][k] * rot[k][j] for k in range(3)) for j in range(3)]
               for i in range(3)]
        frame = joint.find('parent').get('link')
    assert frame == 'base_footprint'
    return rot


def test_camera_image_upright_and_looking_ahead():
    """L'image du pilote est consommee telle quelle : son repere optique doit
    decrire une camera droite. On le verifie sur les axes plutot que sur les
    angles, pour qu'un export retourne ou un mount_rpy oublie se voie."""
    rot = orientation(run_xacro(target='real'), REVISION['camera']['frame'])
    axis = lambda i: [rot[r][i] for r in range(3)]
    x, y, z = axis(0), axis(1), axis(2)
    assert x[1] < -0.99, f'la droite de l image doit etre la droite du robot : x = {x}'
    assert y[2] < -0.5, f'le bas de l image doit pointer vers le sol : y = {y}'
    assert z[0] > 0.5 and z[2] < 0.0, f'l axe optique doit viser devant, vers le sol : z = {z}'


def test_check_urdf():
    urdf = run_xacro(target='real')
    path = Path(os.environ.get('TMPDIR', '/tmp')) / 'alidade_check.urdf'
    path.write_text(ET.tostring(urdf, encoding='unicode'))
    subprocess.run(['check_urdf', str(path)], check=True, capture_output=True)
