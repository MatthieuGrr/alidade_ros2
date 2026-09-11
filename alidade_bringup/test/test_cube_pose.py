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
