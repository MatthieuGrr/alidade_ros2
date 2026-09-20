"""Choix de la pierre a aborder (alidade_bringup/stone.py).

On pose des pierres devant le robot, on calcule ce que le detecteur verrait de
leurs faces, et on verifie qu'il en sort la bonne -- et, quand il n'y en a pas,
qu'aucune n'est choisie plutot qu'une mauvaise.
"""
import math

import numpy as np
import pytest

from alidade_bringup import params, stone

# camera_optical_frame dans base_footprint (revision v1).
CAMERA = np.array([0.1175, 0.0, 0.1039])


@pytest.fixture(scope='module')
def spec():
    return stone.load(params.bringup_share() / 'config' / 'stone.yaml')


@pytest.fixture
def limits():
    return stone.Limits()


def lying(spec, x, y, facing=math.pi, height=None):
    """Marqueur de la face laterale d'une pierre couchee, normale vers `facing`."""
    return (spec.marker_id,
            stone.face_pose((x, y), facing, spec.lying if height is None else height))


def tilted(pose, degrees):
    """Meme marqueur, bascule autour de son axe x : sa normale sort de l'horizontale."""
    a = math.radians(degrees)
    rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    out = pose.copy()
    out[:3, :3] = pose[:3, :3] @ rx
    return out


# ------------------------------------------------------------------ la pierre
def test_hauteurs_deduites_des_cotes(spec):
    assert spec.lying == pytest.approx(0.055)
    assert spec.stacked(1) == pytest.approx(0.055)
    assert spec.stacked(2) == pytest.approx(0.165)
    assert spec.stacked(3) == pytest.approx(0.275)
    assert spec.standing == pytest.approx(0.160)
    # Le cas limite du sujet : 5 mm separent une pierre debout de la deuxieme
    # d'une pile. Aucune fenetre de hauteur ne les distingue, d'ou le
    # regroupement en piles.
    assert abs(spec.standing - spec.stacked(2)) == pytest.approx(0.005)
    assert spec.marker_pad == pytest.approx(0.100)


# ------------------------------------------------------------- redressement
def test_le_redressement_rend_le_cap_de_la_normale(spec):
    for facing in (0.0, math.pi, 1.0, -2.5):
        tag = stone.upright(stone.face_pose((0.5, 0.1), facing, spec.lying), CAMERA)
        assert stone.wrap(tag.yaw - facing) == pytest.approx(0.0, abs=1e-12)
        assert tag.tilt == pytest.approx(0.0, abs=1e-9)
        assert tag.height == pytest.approx(spec.lying)


def test_une_face_penchee_garde_un_cap_juste_jusqu_au_seuil(spec, limits):
    pose = stone.face_pose((0.5, 0.0), math.pi, spec.lying)
    for degrees, expected in ((5.0, None), (24.0, None), (26.0, 'inclinaison'), (40.0, 'inclinaison')):
        tags = stone.assess([(spec.marker_id, tilted(pose, degrees))], spec, CAMERA, limits)
        assert tags[0].reason == expected, degrees
        assert tags[0].tilt == pytest.approx(degrees, abs=1e-6)
        # Le cap reste celui de la projection, quel que soit le basculement.
        assert stone.wrap(tags[0].yaw - math.pi) == pytest.approx(0.0, abs=1e-9)


# ------------------------------------------------------------ pierre isolee
def test_pierre_seule_pose_d_arret_sur_la_normale(spec, limits):
    tags = stone.assess([lying(spec, 0.50, 0.0)], spec, CAMERA, limits)
    target = stone.dock_target(stone.select(tags, limits), 0.22)
    assert (target.x, target.y) == pytest.approx((0.50 - 0.22, 0.0))
    assert target.yaw == pytest.approx(0.0)      # face au marqueur, donc vers +x
    # L'arret est bien a `standoff` de la FACE, pas du centre de la pierre.
    assert math.dist((target.x, target.y), tags[0].centre[:2]) == pytest.approx(0.22)


def test_pierre_de_travers(spec, limits):
    facing = math.radians(200.0)
    tags = stone.assess([lying(spec, 0.45, 0.12, facing)], spec, CAMERA, limits)
    target = stone.dock_target(stone.select(tags, limits), 0.22)
    n = np.array([math.cos(facing), math.sin(facing)])
    assert (target.x, target.y) == pytest.approx(tuple(np.array([0.45, 0.12]) + n * 0.22))
    assert stone.wrap(target.yaw - (facing + math.pi)) == pytest.approx(0.0, abs=1e-12)


# --------------------------------------------------------------------- piles
def test_pile_de_deux_puis_de_trois(spec, limits):
    wide = stone.Limits(height_max=0.400)     # fenetre ouverte : seul le groupement trie
    for levels in (2, 3):
        detections = [lying(spec, 0.50, 0.0, height=spec.stacked(n)) for n in range(1, levels + 1)]
        tags = stone.assess(detections, spec, CAMERA, wide)
        assert [t.reason for t in tags] == [None] + ['empilee'] * (levels - 1)
        assert stone.select(tags, wide).height == pytest.approx(spec.lying)


def test_la_fenetre_de_hauteur_suffit_deja_aux_piles(spec, limits):
    detections = [lying(spec, 0.50, 0.0, height=spec.stacked(n)) for n in (1, 2, 3)]
    tags = stone.assess(detections, spec, CAMERA, limits)
    assert [t.reason for t in tags] == [None, 'hauteur', 'hauteur']


def test_debout_et_deuxieme_d_une_pile_tombent_toutes_deux(spec, limits):
    """Les 160 et 165 mm du sujet : rien ne les separe, les deux sont rejetees."""
    for height in (spec.standing, spec.stacked(2)):
        tags = stone.assess([lying(spec, 0.60, 0.0, height=height)], spec, CAMERA, limits)
        assert tags[0].reason == 'hauteur'
        assert stone.select(tags, limits) is None


def test_une_pierre_debout_seule_ne_passe_pas_meme_fenetre_ouverte(spec):
    """Fenetre ouverte, une pierre debout ISOLEE n'est surplombee par personne :
    c'est bien la fenetre de hauteur qui la rejette, et elle seule. Le jour ou
    le perimetre s'ouvrira aux pierres debout, c'est ce seuil qu'il faudra
    bouger, pas le groupement."""
    wide = stone.Limits(height_max=0.400)
    tags = stone.assess([lying(spec, 0.60, 0.0, height=spec.standing)], spec, CAMERA, wide)
    assert tags[0].reason is None


def test_deux_pierres_cote_a_cote_ne_sont_pas_une_pile(spec, limits):
    detections = [lying(spec, 0.50, -0.09), lying(spec, 0.50, 0.09)]
    tags = stone.assess(detections, spec, CAMERA, limits)
    assert [t.reason for t in tags] == [None, None]


# ------------------------------------------------------- filtres et egalites
def test_identifiant_etranger_signale_et_non_choisi(spec, limits):
    parasite = (7, stone.face_pose((0.40, 0.0), math.pi, spec.lying))
    tags = stone.assess([parasite, lying(spec, 0.60, 0.0)], spec, CAMERA, limits)
    assert [t.reason for t in tags] == ['identifiant', None]
    assert stone.select(tags, limits).centre[0] == pytest.approx(0.60)


def test_face_vue_de_trop_biais(spec, limits):
    for ecart, expected in ((60.0, None), (80.0, 'incidence')):
        facing = math.pi - math.radians(ecart)
        tags = stone.assess([lying(spec, 0.50, 0.0, facing)], spec, CAMERA, limits)
        assert tags[0].reason == expected, ecart
        assert tags[0].incidence == pytest.approx(ecart, abs=0.5)


def test_trop_loin_et_trop_pres(spec, limits):
    assert stone.assess([lying(spec, 1.20, 0.0)], spec, CAMERA, limits)[0].reason == 'distance'
    assert stone.assess([lying(spec, 0.19, 0.0)], spec, CAMERA, limits)[0].reason == 'distance'


def test_a_egalite_de_niveau_la_plus_proche_ou_la_plus_centree(spec):
    # Une pierre proche mais tres decalee, une plus loin mais dans l'axe.
    detections = [lying(spec, 0.35, 0.22), lying(spec, 0.60, 0.01)]
    proche = stone.Limits(tie='proche')
    centre = stone.Limits(tie='centre')
    assert stone.select(stone.assess(detections, spec, CAMERA, proche), proche).centre[0] \
        == pytest.approx(0.35)
    assert stone.select(stone.assess(detections, spec, CAMERA, centre), centre).centre[0] \
        == pytest.approx(0.60)


# ---------------------------------------------------------------- verrouillage
def test_la_cible_verrouillee_ne_bascule_pas(spec, limits):
    loin = lying(spec, 0.60, 0.0)
    tags = stone.assess([loin], spec, CAMERA, limits)
    locked = stone.select(tags, limits).centre
    # Une pierre plus proche entre dans le champ : sans verrou elle gagnerait.
    tags = stone.assess([lying(spec, 0.30, 0.05), loin], spec, CAMERA, limits)
    assert stone.select(tags, limits).centre[0] == pytest.approx(0.30)
    assert stone.select(tags, limits, locked).centre[0] == pytest.approx(0.60)


def test_le_verrou_lache_si_la_cible_disparait(spec, limits):
    locked = np.array([0.60, 0.0, spec.lying])
    tags = stone.assess([lying(spec, 0.30, 0.05)], spec, CAMERA, limits)
    assert stone.select(tags, limits, locked).centre[0] == pytest.approx(0.30)


def test_rien_a_viser_ne_rend_rien(spec, limits):
    assert stone.select([], limits) is None
    tags = stone.assess([lying(spec, 1.20, 0.0)], spec, CAMERA, limits)
    assert stone.select(tags, limits) is None
