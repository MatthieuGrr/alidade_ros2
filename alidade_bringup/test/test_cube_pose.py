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


# ----------------------------------------------------------------- cube au sol, orientations faussees
CAMERA = np.array([0.1175, 0.0, 0.104])     # repere optique dans base_footprint


def on_floor(yaw_deg, x=0.6, y=0.1, lying=None):
    """Pose d'un cube au sol dans le repere du robot ; `lying` : rotation
    supplementaire pour le poser sur une autre face."""
    pose = np.eye(4)
    rot = rot_z(yaw_deg)
    if lying is not None:
        rot = rot @ lying
    pose[:3, :3] = rot
    pose[:3, 3] = [x, y, 0.025]
    return pose


def markers_of(cube_pose, model, ids, twist_deg=0.0):
    """Poses des marqueurs ; `twist_deg` fausse leur orientation autour de la
    verticale en gardant leur position, comme le fait le detecteur."""
    _, faces = model
    out = {}
    for i in ids:
        m = cube_pose @ faces[i].pose
        m[:3, :3] = rot_z(twist_deg) @ m[:3, :3]
        out[i] = m
    return out


def yaw_error(rot_a, rot_b):
    return np.degrees(np.arccos(np.clip((np.trace(rot_a.T @ rot_b) - 1) / 2, -1, 1)))


def test_deux_faces_voisines_orientations_fausses(model):
    edge, faces = model
    truth = on_floor(142)                               # le cas mesure sur le robot
    # Chaque face se trompe a sa facon, comme sur le robot (+60 et -40 deg).
    seen = {**markers_of(truth, model, [2], twist_deg=60), **markers_of(truth, model, [5], twist_deg=-40)}
    naive = cube.estimate(seen, faces)
    assert yaw_error(naive.rotation, truth[:3, :3]) > 5      # les orientations seules se trompent
    assert naive.angle > 30                                   # et se contredisent
    est = cube.estimate_on_floor(seen, faces, edge, CAMERA)
    assert est.method == 'geometrie'
    assert np.allclose(est.position, truth[:3, 3], atol=1e-9)
    assert yaw_error(est.rotation, truth[:3, :3]) < 1e-6


def test_une_face_et_le_dessus_orientations_fausses(model):
    edge, faces = model
    truth = on_floor(-20, x=0.35, y=0.0)                # face 4 vers le robot, dessus 1
    seen = markers_of(truth, model, [1, 4], twist_deg=-45)
    est = cube.estimate_on_floor(seen, faces, edge, CAMERA)
    assert est.method == 'geometrie'
    assert np.allclose(est.position, truth[:3, 3], atol=1e-9)
    assert yaw_error(est.rotation, truth[:3, :3]) < 1e-6


def test_cube_couche_sur_une_autre_face(model):
    """Rien ne suppose quelle face est dessus : c'est la hauteur mesuree qui le dit."""
    edge, faces = model
    truth = on_floor(30, x=0.5, y=0.05, lying=rot_x(90))   # face 5 (+y) passe dessus
    _, f = model
    heights = {i: (truth @ f[i].pose)[2, 3] for i in f}
    top = max(heights, key=heights.get)
    visibles = [i for i in f if (truth @ f[i].pose)[:3, 2] @ (CAMERA - (truth @ f[i].pose)[:3, 3]) > 0.01]
    sides = [i for i in visibles if i != top and abs(heights[i] - 0.025) < 1e-6][:2]
    est = cube.estimate_on_floor(markers_of(truth, model, sides, twist_deg=50), faces, edge, CAMERA)
    assert est.method == 'geometrie'
    assert yaw_error(est.rotation, truth[:3, :3]) < 1e-6


def test_une_seule_face_repli_sur_l_orientation(model):
    edge, faces = model
    truth = on_floor(10, x=0.3, y=0.0)
    est = cube.estimate_on_floor(markers_of(truth, model, [4]), faces, edge, CAMERA)
    assert est.method == 'orientation'
    assert yaw_error(est.rotation, truth[:3, :3]) < 1e-6


def test_face_seule_de_loin_pas_de_pose(model):
    edge, faces = model
    loin = on_floor(10, x=0.9, y=0.0)
    assert cube.estimate_on_floor(markers_of(loin, model, [4]), faces, edge, CAMERA,
                                  single_face_range=0.45) is None
    pres = on_floor(10, x=0.35, y=0.0)
    est = cube.estimate_on_floor(markers_of(pres, model, [4]), faces, edge, CAMERA, single_face_range=0.45)
    assert est is not None and est.method == 'orientation'
    # deux faces de loin : la geometrie reste valable
    deux = cube.estimate_on_floor(markers_of(on_floor(142, x=0.9), model, [2, 5]), faces, edge, CAMERA,
                                  single_face_range=0.45)
    assert deux is not None and deux.method == 'geometrie'


def test_appariement_incoherent_ignore(model):
    """Deux marqueurs trop eloignes pour etre deux faces voisines du meme cube."""
    edge, faces = model
    seen = markers_of(on_floor(142), model, [2, 5])
    seen[5] = seen[5].copy()
    seen[5][:3, 3] += [0.05, 0.0, 0.0]
    assert cube.estimate_on_floor(seen, faces, edge, CAMERA).method == 'orientation'
