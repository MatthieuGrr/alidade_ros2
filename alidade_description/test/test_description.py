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
# La revision de reference est la DERNIERE presente, pas un nom code en dur :
# renommer une revision ne doit pas casser la collecte des tests.
REVISION_FILE = sorted((PKG / 'config').glob('alidade_v*.yaml'))[-1]
REVISION = yaml.safe_load(REVISION_FILE.read_text())


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


def orientation(robot, frame, racine='base_footprint'):
    """Orientation de frame dans la racine, en remontant les joints.

    `racine` vaut 'root' sur un export Onshape brut, que fix_urdf.py n'a pas
    encore renomme : c'est ce qui permet de comparer l'avant et l'apres.
    """
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
    assert frame == racine, f'{frame} ne remonte pas a {racine}'
    return rot


def test_check_urdf():
    urdf = run_xacro(target='real')
    path = Path(os.environ.get('TMPDIR', '/tmp')) / 'alidade_check.urdf'
    path.write_text(ET.tostring(urdf, encoding='unicode'))
    subprocess.run(['check_urdf', str(path)], check=True, capture_output=True)



# --------------------------------------------------------------- revisions
# Le materiel se decrit revision par revision (config/alidade_v<n>.yaml), et
# `CMakeLists.txt` les ramasse au glob : en ajouter une est un simple fichier.
# Le risque est alors qu'une mesure -- calibration d'odometrie, masse, entraxe
# -- soit reportee dans une seule. Ce controle-la se paie une fois ici, et vaut
# pour toutes les revisions presentes, y compris quand il n'y en a qu'une.


def flatten(node, prefix=''):
    if isinstance(node, dict):
        for key, value in node.items():
            yield from flatten(value, f'{prefix}.{key}' if prefix else str(key))
    else:
        yield prefix, node


def revisions() -> dict[str, dict]:
    return {p.stem.split('_')[-1]: dict(flatten(yaml.safe_load(p.read_text())))
            for p in sorted((PKG / 'config').glob('alidade_v*.yaml'))}


def test_toutes_les_revisions_decrivent_les_memes_cles():
    """Une cle presente dans une seule revision est presque toujours un oubli.

    Une nouvelle revision se fait en copiant la precedente et en changeant ce
    qui a bouge physiquement ; ce qui n'a pas bouge doit rester identique, et
    une mesure refaite doit etre reportee partout.
    """
    revs = revisions()
    assert revs, 'aucune revision dans config/'
    reference, cles = next(iter(revs.items())), None
    for nom, valeurs in revs.items():
        if cles is None:
            cles = set(valeurs)
            continue
        manquantes, en_trop = cles - set(valeurs), set(valeurs) - cles
        assert not (manquantes or en_trop), (
            f'{nom} ne decrit pas les memes cles que {reference[0]} : '
            f'manque {sorted(manquantes)}, en trop {sorted(en_trop)}')


def test_les_corrections_de_montage_sont_appliquees():
    """Toute section declarant `frame` et `mount_rpy` doit voir sa correction.

    La CAO place un capteur mais ignore ou tombe son repere de MESURE : le 0
    d un LiDAR, l axe optique d une camera dans son boitier. Ces corrections se
    declarent dans la revision, et leur oubli ne se voit nulle part ailleurs --
    un demi-tour manquant sur le LiDAR fait cartographier la piece a l envers
    sans le moindre message. On compare donc l export BRUT a l URDF corrige.
    """
    for revision in revisions():
        cfg = yaml.safe_load((PKG / 'config' / f'alidade_{revision}.yaml').read_text())
        brut = ET.parse(PKG / 'urdf' / cfg['onshape_export']).getroot()
        corrige = run_xacro(target='real', hardware_revision=revision)
        verifiees = 0
        for section, spec in cfg.items():
            if not isinstance(spec, dict) or 'frame' not in spec or 'mount_rpy' not in spec:
                continue
            avant = orientation(brut, spec['frame'], racine='root')
            apres = orientation(corrige, spec['frame'])
            attendu = [[sum(avant[i][k] * rotation(spec['mount_rpy'])[k][j] for k in range(3))
                        for j in range(3)] for i in range(3)]
            ecart = max(abs(apres[i][j] - attendu[i][j]) for i in range(3) for j in range(3))
            assert ecart < 1e-6, (
                f'{revision}/{section} : la correction mount_rpy de {spec["frame"]} '
                f'n est pas appliquee (ecart {ecart:.2e})')
            verifiees += 1
        assert verifiees, f'{revision} : aucune correction de montage a verifier'
