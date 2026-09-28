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


# --------------------------------------------------------------- revisions
# v2 est la configuration Coupe de France : elle ne doit differer de v1 QUE par
# le support de camera. Sans ce controle, une calibration d'odometrie ou une
# masse reportee dans un seul des deux fichiers passerait inapercue -- c'est le
# prix a payer pour deux revisions, et il se paie une fois ici.
DIFFERENCES = {
    # Chaque revision derive de la precedente et ne doit en differer que par ce
    # qui est liste ici. Sans ce controle, une calibration d'odometrie reportee
    # dans un seul fichier passerait inapercue -- c'est le prix de plusieurs
    # revisions, et il se paie une fois.
    ('v1', 'v2'): ({'revision', 'camera.mount_rpy'}, set()),
    ('v2', 'v3'): ({'revision', 'frames.expected.laser', 'lidar.mount_rpy',
                    'lidar.samples_per_rev'},
                   {'lidar.mount_xyz', 'lidar.fov_min', 'lidar.fov_max'}),
}


def flatten(node, prefix=''):
    if isinstance(node, dict):
        for key, value in node.items():
            yield from flatten(value, f'{prefix}.{key}' if prefix else str(key))
    else:
        yield prefix, node


def revision(nom):
    plat = dict(flatten(yaml.safe_load((PKG / 'config' / f'alidade_{nom}.yaml').read_text())))
    # mount_rpy et les poses sont des listes : flatten les reduit a cle.0, .1...
    groupe = {}
    for k, v in plat.items():
        tete, _, fin = k.rpartition('.')
        groupe.setdefault(tete if fin.isdigit() else k, []).append(v)
    return groupe


@pytest.mark.parametrize('avant,apres', list(DIFFERENCES))
def test_une_revision_ne_differe_que_de_ce_qui_est_declare(avant, apres):
    attendu, nouvelles = DIFFERENCES[(avant, apres)]
    a, b = revision(avant), revision(apres)
    assert set(b) - set(a) == nouvelles, f'{apres} : cles ajoutees inattendues'
    assert set(a) - set(b) == set(), f'{apres} : cles disparues'
    differentes = {k for k in a if a[k] != b[k]}
    assert differentes == attendu, (
        f'{avant} -> {apres} different sur {sorted(differentes)} ; attendu {sorted(attendu)}. '
        'Reporter la mesure dans les deux fichiers, ou mettre a jour DIFFERENCES.')


def test_v2_plonge_de_quinze_degres():
    """L'inclinaison decide de la portee sur les pierres : au-dela de 20.4 deg
    (demi-ouverture verticale), le marqueur d'une pierre couchee sort du haut
    de l'image. Voir alidade_bringup/stone.py et le README."""
    for rev, expected in (('v1', 23.5), ('v2', 15.0), ('v3', 15.0)):
        rot = orientation(run_xacro(target='real', hardware_revision=rev),
                          REVISION['camera']['frame'])
        # z du repere optique = axe de visee ; sa composante verticale donne la plongee.
        plongee = math.degrees(math.asin(-rot[2][2]))
        assert abs(plongee - expected) < 0.3, f'{rev} : plongee {plongee:.2f} deg'


def test_v3_voit_les_bordures_de_la_table():
    """Le plan de mesure doit passer SOUS les bordures de 70 mm, sans quoi toute
    la revision v3 ne sert a rien. Le plan est a 29.8 mm au-dessus de la base du
    C1 (fiche Slamtec) : c'est la cote qui rend la manoeuvre possible."""
    v3 = yaml.safe_load((PKG / 'config' / 'alidade_v3.yaml').read_text())
    z = v3['frames']['expected']['laser'][2]
    assert z < 0.070, f'plan de mesure a {z*1000:.0f} mm : au-dessus de la bordure'
    assert z - 0.0298 > 0.005, 'la base du C1 passerait sous le plateau'
    x = v3['frames']['expected']['laser'][0]
    assert x > v3['kinematics']['wheel_separation'] * 0, ''
    # En avant des roues, dont le sommet est a 2 x le rayon.
    assert z < 2 * v3['kinematics']['wheel_radius'] or x > 0.10, \
        'a cette hauteur le capteur doit etre en avant des roues'
    assert math.degrees(v3['lidar']['fov_max'] - v3['lidar']['fov_min']) == pytest.approx(180.0)
    # 5000 points/s a 10 Hz : le champ reduit n'en cree aucun.
    assert v3['lidar']['samples_per_rev'] == pytest.approx(
        5000 / v3['lidar']['rotation_hz'] * 180 / 360)
