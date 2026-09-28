"""Recalage sur les bordures : geometrie, covariance, et veto de cap.

Les scans sont synthetiques -- un lancer de rayon sur le rectangle -- pour que
ces tests tournent sans Gazebo et sans ROS, et qu'un echec designe le module et
rien d'autre.
"""
import math

import numpy as np
import pytest

from alidade_bringup import borders


TABLE = borders.Table()


def scan(pose, n=250, demi_champ=math.pi / 2, bruit=0.0, seed=1, obstacles=()):
    """Scan simule depuis `pose` : distance au premier obstacle, puis au mur.

    `obstacles` : des segments (x1, y1, x2, y2) poses sur la table, de quoi
    figurer la face d'une pierre.
    """
    rng = np.random.default_rng(seed)
    x, y, yaw = pose
    normales, decalages = TABLE.walls()
    pts = []
    for a in np.linspace(-demi_champ, demi_champ, n):
        u = np.array([math.cos(a + yaw), math.sin(a + yaw)])
        portees = []
        for nn, dd in zip(normales, decalages):
            den = float(nn @ u)
            if den < -1e-9:
                portees.append(-(float(nn @ (x, y)) + dd) / den)
        for (x1, y1, x2, y2) in obstacles:
            seg = np.array([x2 - x1, y2 - y1])
            det = u[0] * -seg[1] - u[1] * -seg[0]
            if abs(det) < 1e-12:
                continue
            dxy = np.array([x1 - x, y1 - y])
            t = (dxy[0] * -seg[1] - dxy[1] * -seg[0]) / det
            v = (u[0] * dxy[1] - u[1] * dxy[0]) / det
            if t > 0 and 0.0 <= v <= 1.0:
                portees.append(t)
        r = min(portees) + (rng.normal(0.0, bruit) if bruit else 0.0)
        pts.append((r * math.cos(a), r * math.sin(a)))
    return np.array(pts)


def decale(pose, dx=0.004, dy=0.004, dc=0.004):
    return pose[0] + dx, pose[1] + dy, pose[2] + dc


# ------------------------------------------------------------------ geometrie

def test_le_signe_des_murs_lointains_est_positif():
    """Le bug qui imite une singularite : e doit s'annuler SUR chaque mur."""
    n, d = TABLE.walls()
    for p, attendu in (((0.0, 1.0), 0), ((3.0, 1.0), 1), ((1.5, 0.0), 2), ((1.5, 2.0), 3)):
        e = np.asarray(p) @ n.T + d
        assert abs(e[attendu]) < 1e-12, f'le mur {attendu} ne passe pas par {p}'
        assert np.all(e >= -1e-12), 'aucune distance interieure ne doit etre negative'


def test_scan_parfait_retrouve_la_pose_exactement():
    vrai = (1.5, 1.0, 0.0)
    fix = borders.match(scan(vrai), decale(vrai))
    assert fix.ok
    assert math.dist((fix.x, fix.y), vrai[:2]) < 1e-4
    assert abs(borders.wrap(fix.yaw - vrai[2])) < 1e-5


def test_les_quatre_murs_sont_utilises():
    """Un tour complet au centre : tous les points servent, aucun n'est orphelin."""
    fix = borders.match(scan((1.5, 1.0, 0.0), demi_champ=math.pi), (1.5, 1.0, 0.0))
    assert fix.ok
    assert fix.inliers >= 240


@pytest.mark.parametrize('pose', [(0.6, 0.6, 0.0), (2.4, 1.4, math.pi / 2),
                                  (1.5, 0.6, math.pi), (2.4, 0.6, -math.pi / 2)])
def test_bruit_realiste_reste_sous_le_centimetre(pose):
    """30 mm d'ecart-type par point -- la fiche du C1 -- sur 250 points."""
    fix = borders.match(scan(pose, bruit=0.030, seed=7), decale(pose))
    assert fix.ok
    assert math.dist((fix.x, fix.y), pose[:2]) < 0.010
    assert abs(borders.wrap(fix.yaw - pose[2])) < math.radians(0.5)


# ------------------------------------------------------- symetrie de la table

def test_la_pose_jumelle_recale_aussi_bien():
    """Le fait desagreable, ecrit noir sur blanc : le scan ne tranche pas.

    Si ce test se met a echouer, ce n'est pas une bonne nouvelle : c'est que le
    modele de table a cesse d'etre symetrique, et tout ce qui suit est a revoir.
    """
    vrai = (0.6, 0.6, 0.0)
    pts = scan(vrai, bruit=0.010, seed=3)
    bon = borders.match(pts, decale(vrai))
    jumeau = borders.match(pts, decale(borders.mirror(vrai)))
    assert bon.ok and jumeau.ok
    assert math.dist((jumeau.x, jumeau.y), borders.mirror(vrai)[:2]) < 0.010
    # Meme qualite d'ajustement : aucune information ne separe les deux.
    assert abs(bon.inliers - jumeau.inliers) <= 2


def test_mirror_est_une_involution():
    pose = (0.7, 1.3, 1.0)
    retour = borders.mirror(borders.mirror(pose))
    assert math.dist(retour[:2], pose[:2]) < 1e-12
    assert abs(borders.wrap(retour[2] - pose[2])) < 1e-12


# ------------------------------------------------------------- veto de cap

def test_le_veto_refuse_la_pose_jumelle():
    vrai = (0.6, 0.6, 0.0)
    pts = scan(vrai, bruit=0.010, seed=3)
    fix = borders.match(pts, decale(borders.mirror(vrai)), heading=vrai[2])
    assert not fix.ok
    assert fix.reason == 'cap'


def test_le_veto_laisse_passer_la_bonne_pose():
    vrai = (0.6, 0.6, 0.0)
    fix = borders.match(scan(vrai, bruit=0.010, seed=3), decale(vrai), heading=vrai[2])
    assert fix.ok


def test_le_veto_tolere_un_cap_tres_approximatif():
    """Le gyromètre derive de 0.3 deg par minute ; on lui en accorde 80."""
    vrai = (1.5, 1.0, 0.0)
    fix = borders.match(scan(vrai, bruit=0.010, seed=5), decale(vrai),
                        heading=math.radians(80))
    assert fix.ok, 'un cap grossier ne doit pas faire rejeter une bonne pose'


def test_sans_cap_de_reference_le_veto_ne_s_applique_pas():
    vrai = (0.6, 0.6, 0.0)
    pts = scan(vrai, bruit=0.010, seed=3)
    assert borders.match(pts, decale(borders.mirror(vrai))).ok


# --------------------------------------------------- garde-fous et covariance

@pytest.mark.parametrize('vrai,pierre', [
    ((1.5, 0.6, -math.pi / 2), (1.34, 0.25, 1.66, 0.25)),
    ((0.6, 1.0, math.pi), (0.25, 0.84, 0.25, 1.16)),
    ((1.5, 1.4, math.pi / 2), (1.34, 1.75, 1.66, 1.75)),
])
def test_une_face_de_pierre_ne_produit_jamais_une_erreur_tue(vrai, pierre):
    """Le contrat du module : ne jamais se tromper AVEC ASSURANCE.

    Une pile de trois pierres offre 320 mm de face plane, parallele a un mur et
    a 350 mm de lui : le recalage peut s'y accrocher et glisser de 345 mm --
    mesure. Ce qui est interdit, ce n'est pas de se tromper, c'est de se tromper
    en annoncant un millimetre. On exige donc que chaque axe faux soit couvert
    par le sigma annonce sur cet axe, ou que la pose soit refusee.
    """
    pts = scan(vrai, bruit=0.010, seed=11, obstacles=[pierre])
    fix = borders.match(pts, (vrai[0] + 0.30, vrai[1] - 0.30, vrai[2]))
    if not fix.ok:
        return
    sigmas = fix.sigma()
    erreurs = (abs(fix.x - vrai[0]), abs(fix.y - vrai[1]),
               abs(borders.wrap(fix.yaw - vrai[2])))
    for axe, (err, sig) in enumerate(zip(erreurs, sigmas)):
        assert err <= 3 * sig + 0.020, (
            f"axe {axe} : erreur {err:.3f} annoncee a {sig:.4f} -- faux et sur de lui")


def test_trop_peu_de_points_est_signale_et_non_devine():
    fix = borders.match(np.zeros((5, 2)), (1.5, 1.0, 0.0))
    assert fix.reason == 'points'
    assert fix.inliers == 0


def test_un_seul_mur_visible_rend_un_axe_a_covariance_enorme():
    """Face a un mur de pres : la distance et le cap sont surs, le glissement non."""
    vrai = (2.9, 1.0, 0.0)
    fix = borders.match(scan(vrai, demi_champ=math.radians(40), bruit=0.005),
                        decale(vrai))
    assert fix.ok
    sx, sy, scap = fix.sigma()
    assert sx < 0.005, 'la distance au mur doit rester mesuree'
    assert scap < math.radians(0.5), 'le cap doit rester mesure'
    assert sy > 1.0, "le glissement le long du mur n'est pas observable"


def test_une_bonne_geometrie_annonce_quelques_millimetres():
    fix = borders.match(scan((1.5, 1.0, 0.0), bruit=0.030, seed=2), (1.5, 1.0, 0.0))
    assert fix.ok
    sx, sy, scap = fix.sigma()
    assert sx < 0.010 and sy < 0.010
    assert scap < math.radians(0.5)
