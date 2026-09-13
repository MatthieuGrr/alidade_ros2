"""Commande du docking (alidade_bringup/docking.py) sur un robot cinematique.

On rejoue mise en place puis approche depuis une grille de departs realistes
-- ceux ou l'operateur laisse le robot, cube dans le champ -- et on verifie
l'arrivee : sur la ligne d'arret, dans l'axe, bien oriente, sans l'avoir
jamais depassee.
"""
import itertools
import math

import pytest

from alidade_bringup import docking as d

DT = 0.05   # 20 Hz, la cadence du noeud


TRACK = 0.1939       # entraxe (revision v1)
DEADBAND = 0.03      # m/s par roue : en dessous, la roue ne tourne pas (mesure)


def simulate(start, dock, g, lag=0.0, deadband=False, limit=40.0):
    """Robot unicycle ; `lag` : fraction de la commande precedente conservee
    (inertie grossiere de la chaine lisseur + moteurs) ; `deadband` : chaque
    roue commandee sous DEADBAND reste immobile, comme mesure sur le robot."""
    robot = start
    stager = d.Stager(dock.ahead(-g.staging), g) if d.staging_needed(robot, dock, g) else None
    v_prev = w_prev = 0.0
    furthest = -math.inf
    t = 0.0
    while t < limit:
        if stager is not None:
            v, w, done = stager.step(robot)
            if done:
                stager = None
                continue
        else:
            v, w, done = d.approach(robot, dock, g)
            if done:
                return robot, furthest, t
        v = lag * v_prev + (1 - lag) * v
        w = lag * w_prev + (1 - lag) * w
        v_prev, w_prev = v, w
        if deadband:
            left, right = v - w * TRACK / 2, v + w * TRACK / 2
            left, right = (0.0 if abs(x) < DEADBAND else x for x in (left, right))
            v, w = (left + right) / 2, (right - left) / TRACK
        robot = d.Pose2D(robot.x + v * math.cos(robot.yaw) * DT,
                         robot.y + v * math.sin(robot.yaw) * DT,
                         robot.yaw + w * DT)
        if stager is None:
            furthest = max(furthest, d.errors(robot, dock)[0])
        t += DT
    pytest.fail(f'pas arrive en {limit} s depuis {start}')


DOCK = d.Pose2D(0.0, 0.0, 0.0)
# Distance a la pose d'arret 10-30 cm, decalage +-6 cm, cap +-45 degres : la
# face la plus en face fait jusqu'a 45 degres avec la ligne de visee.
STARTS = [d.Pose2D(-dist, lat, math.radians(hdg))
          for dist, lat, hdg in itertools.product([0.10, 0.20, 0.30], [-0.06, -0.02, 0.0, 0.03, 0.06],
                                                  [-45, -20, 0, 15, 45])]


@pytest.mark.parametrize('lag,deadband', [(0.0, False), (0.5, False), (0.5, True)])
def test_arrivee_sur_la_ligne_dans_l_axe(lag, deadband):
    g = d.Gains()
    for start in STARTS:
        end, furthest, _ = simulate(start, DOCK, g, lag=lag, deadband=deadband)
        along, lat, yaw = d.errors(end, DOCK)
        assert abs(lat) <= g.lat_tol, f'{start} : lateral {lat * 100:.1f} cm'
        assert abs(math.degrees(yaw)) <= math.degrees(g.arrive_yaw_tol), f'{start} : cap {math.degrees(yaw):.1f}'
        assert -0.005 <= along <= 0.01, f'{start} : arret a {along * 100:.1f} cm de la ligne'
        assert furthest <= 0.01, f'{start} : ligne depassee de {furthest * 100:.1f} cm'


def test_deja_en_place_on_saute_la_mise_en_place():
    g = d.Gains()
    assert not d.staging_needed(d.Pose2D(-0.20, 0.005, math.radians(2)), DOCK, g)
    assert d.staging_needed(d.Pose2D(-0.20, 0.04, 0.0), DOCK, g)            # decale
    assert d.staging_needed(d.Pose2D(-0.20, 0.0, math.radians(20)), DOCK, g)  # de travers
    assert d.staging_needed(d.Pose2D(-0.05, 0.0, 0.0), DOCK, g)             # trop pres


def test_cube_trop_pres_le_robot_recule_d_abord():
    g = d.Gains()
    stager = d.Stager(DOCK.ahead(-g.staging), g)
    v, w, done = stager.step(d.Pose2D(-0.05, 0.0, 0.0))
    # pivot eventuel sans avancer, puis la ligne droite se fait en arriere
    robot = d.Pose2D(-0.05, 0.0, 0.0)
    for _ in range(200):
        v, w, done = stager.step(robot)
        if v != 0.0:
            break
        robot = d.Pose2D(robot.x, robot.y, robot.yaw + w * DT)
    assert v < 0.0


def test_fin_a_l_aveugle_seulement_pres_de_la_ligne():
    g = d.Gains(blind=0.04)
    assert d.may_finish_blind(d.Pose2D(-0.03, 0.0, 0.0), DOCK, g)     # 3 cm : on finit
    assert d.may_finish_blind(d.Pose2D(0.002, 0.0, 0.0), DOCK, g)     # deja sur la ligne
    assert not d.may_finish_blind(d.Pose2D(-0.06, 0.0, 0.0), DOCK, g)  # 6 cm : cube perdu = echec
    # l'ecart lateral ne compte pas : c'est la distance a parcourir qui decide
    assert d.may_finish_blind(d.Pose2D(-0.03, 0.05, 0.0), DOCK, g)


def test_approche_s_arrete_sur_la_ligne_meme_decalee():
    v, w, done = d.approach(d.Pose2D(0.001, 0.03, 0.0), DOCK, d.Gains())
    assert done and v == 0.0 and w == 0.0
    assert not d.arrived_ok(d.Pose2D(0.001, 0.03, 0.0), DOCK, d.Gains())


def test_errors_dans_le_repere_de_la_pose_d_arret():
    dock = d.Pose2D(1.0, 1.0, math.pi / 2)
    along, lat, yaw = d.errors(d.Pose2D(1.02, 0.8, math.pi / 2), dock)
    assert along == pytest.approx(-0.2)
    assert lat == pytest.approx(-0.02)
    assert yaw == pytest.approx(0.0)


# ----------------------------------------------------------------- filtre de la cible
def test_filtre_attenue_le_bruit_de_cap_sans_biais():
    import random
    rng = random.Random(1)
    f = d.TargetFilter()
    lever, yaw = 0.245, math.radians(20)
    lats = []
    for i in range(60):
        f.update((0.45, 0.05), yaw + math.radians(rng.choice([-8, 8]) + rng.gauss(0, 1)), lever)
        if i >= 20:
            t = f.target()
            true = d.Pose2D(0.45 - lever * math.cos(yaw), 0.05 - lever * math.sin(yaw), yaw)
            lats.append(abs(d.errors(t, true)[1]))
    # Brut, +-8 degres au bout de 24.5 cm font 3.4 cm ; filtre, moins d'un.
    assert max(lats) < 0.01


def test_filtre_ecarte_une_aberration_isolee():
    f = d.TargetFilter()
    for _ in range(10):
        f.update((0.4, 0.0), 0.0, 0.245)
    f.update((0.4, 0.0), math.radians(90), 0.245)
    assert abs(f.target().yaw) < 1e-9


def test_filtre_accepte_un_changement_de_face_qui_persiste():
    f = d.TargetFilter(switch_count=5)
    for _ in range(10):
        f.update((0.4, 0.0), 0.0, 0.245)
    for _ in range(5):
        f.update((0.4, 0.0), math.radians(90), 0.245)
    assert f.target().yaw == pytest.approx(math.radians(90))
    assert f.target().x == pytest.approx(0.4, abs=1e-9)
    assert f.target().y == pytest.approx(-0.245)
