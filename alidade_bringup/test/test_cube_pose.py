"""Pose du cube depuis ses faces (alidade_bringup/cube.py).

On pose un cube quelque part devant la camera, on calcule ce que le
detecteur verrait de chacune de ses faces, et on verifie que chaque face,
seule ou avec d'autres, rend la pose de depart.
"""
import itertools

import numpy as np
import pytest

from alidade_bringup import cube, params


@pytest.fixture(scope='module')
def model():
    return cube.load(params.bringup_share() / 'config' / 'cube.yaml')


def rot_z(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])


def rot_x(deg):
    a = np.radians(deg)
    return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])


@pytest.fixture(scope='module')
def truth():
    """Cube a 30 cm devant une camera optique (z devant, y en bas), tourne."""
    pose = np.eye(4)
    pose[:3, :3] = rot_x(-100) @ rot_z(35)
    pose[:3, 3] = [0.02, 0.05, 0.30]
    return pose


def seen(truth, model, ids):
    _, faces = model
    return {i: truth @ faces[i].pose for i in ids}


def test_chaque_face_seule_rend_la_pose(model, truth):
    _, faces = model
    for i in faces:
        est = cube.estimate(seen(truth, model, [i]), faces)
        assert np.allclose(est.position, truth[:3, 3], atol=1e-9), f'face {i}'
        assert np.allclose(est.rotation, truth[:3, :3], atol=1e-9), f'face {i}'


def test_plusieurs_faces_s_accordent(model, truth):
    _, faces = model
    for ids in itertools.combinations(faces, 3):
        est = cube.estimate(seen(truth, model, ids), faces)
        assert est.faces == sorted(ids)
        assert est.spread < 1e-9
        assert est.angle < 1e-6
        assert np.allclose(est.position, truth[:3, 3], atol=1e-9)


def test_le_centre_est_a_une_demi_arete_derriere_chaque_marqueur(model, truth):
    edge, faces = model
    for i, m in seen(truth, model, faces).items():
        centre = cube.estimate({i: m}, faces).position
        assert np.isclose(np.linalg.norm(centre - m[:3, 3]), edge / 2)
        # et du cote oppose a la normale du marqueur (son z)
        assert (centre - m[:3, 3]) @ m[:3, 2] < 0


def test_face_mal_decrite_se_voit_a_l_angle_pas_a_la_position(model, truth):
    """Motif tourne d'un quart de tour sur sa face : le centre ne bouge pas
    (il est sur la normale), seule l'orientation diverge."""
    _, faces = model
    markers = seen(truth, model, [1, 4])
    quart = np.eye(4)
    quart[:3, :3] = rot_z(90)
    markers[1] = markers[1] @ quart
    est = cube.estimate(markers, faces)
    assert est.spread < 1e-9
    assert est.angle == pytest.approx(45.0, abs=1e-6)


def test_ids_etrangers_ignores(model, truth):
    _, faces = model
    markers = seen(truth, model, [4])
    markers[17] = np.eye(4)
    assert cube.estimate(markers, faces).faces == [4]
    assert cube.estimate({17: np.eye(4)}, faces) is None


def test_quaternion_aller_retour():
    for rot in (np.eye(3), rot_z(179), rot_x(-100) @ rot_z(35), rot_x(180)):
        q = cube.quaternion(rot)
        assert np.isclose(np.linalg.norm(q), 1.0)
        assert np.allclose(cube.matrix([0, 0, 0], q)[:3, :3], rot, atol=1e-9)


# ----------------------------------------------------------------- pose d'arret
STANDOFF = 0.22   # 10 cm d'ecart + 12 cm de base_footprint a la face avant


def yawed(deg, x=0.45, y=0.0):
    return np.array([x, y, 0.025]), rot_z(deg)


def test_sides_conformes_a_cube_yaml(model):
    _, faces = model
    for (col, sign), fid in cube.SIDES.items():
        expected = np.zeros(3)
        expected[col] = sign
        assert np.allclose(faces[fid].pose[:3, 2], expected), f'face {fid}'


def test_cube_droit_devant(model):
    edge, _ = model
    t = cube.dock_target(*yawed(0), edge, STANDOFF)
    assert t.face_id == 4                       # sa face -x regarde le robot
    assert t.x == pytest.approx(0.45 - edge / 2 - STANDOFF)
    assert t.y == pytest.approx(0.0, abs=1e-12)
    assert t.yaw == pytest.approx(0.0, abs=1e-12)


def test_cube_tourne_le_robot_s_aligne_sur_la_face(model):
    edge, _ = model
    t = cube.dock_target(*yawed(30), edge, STANDOFF)
    assert t.face_id == 4
    assert np.degrees(t.yaw) == pytest.approx(30.0)
    # a l'arret, le centre du cube est droit devant, a demi-arete + standoff
    ahead = np.array([np.cos(t.yaw), np.sin(t.yaw)])
    centre = np.array([0.45, 0.0])
    assert (centre - [t.x, t.y]) @ ahead == pytest.approx(edge / 2 + STANDOFF)
    assert np.cross(ahead, centre - [t.x, t.y]) == pytest.approx(0.0, abs=1e-12)


def test_hysteresis_entre_deux_faces(model):
    edge, _ = model
    # Tourne de +50 degres, le cube presente sa face +y (la 5) a 40 degres du
    # robot, contre 50 pour la face 4...
    assert cube.dock_target(*yawed(50), edge, STANDOFF).face_id == 5
    # ... mais on garde la face 4 si c'etait deja la cible : 10 < 15 degres.
    assert cube.dock_target(*yawed(50), edge, STANDOFF, previous=(0, -1)).face_id == 4
    # A 70 degres, l'ecart (70 - 20 = 50) l'emporte sur l'hysteresis.
    assert cube.dock_target(*yawed(70), edge, STANDOFF, previous=(0, -1)).face_id == 5


def test_cube_decale_la_face_choisie_regarde_le_robot(model):
    edge, _ = model
    # Cube a gauche devant : c'est sa face -x qui regarde le robot, et le cap
    # d'arrivee reste celui de la face, pas celui du rayon robot -> cube.
    t = cube.dock_target(*yawed(0, x=0.40, y=0.15), edge, STANDOFF)
    assert t.face_id == 4 and t.yaw == pytest.approx(0.0, abs=1e-12)
    assert t.y == pytest.approx(0.15)
