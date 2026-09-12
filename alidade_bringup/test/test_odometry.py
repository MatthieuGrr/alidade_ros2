"""Odometrie a cap gyroscopique (alidade_bringup/odometry.py).

On rejoue des trajets connus au pas du robot (50 Hz) et on verifie la pose
d'arrivee, puis les cas qui comptent : gyromètre absent, trame de zeros,
biais a estimer, echelle a corriger.
"""
import math

import pytest

from alidade_bringup import odometry as o

DT = 0.02       # 50 Hz, la cadence des trames du Pico


def run(steps, dt=DT, gains=None):
    """steps : liste de (v, w_roues, w_gyro) a appliquer, un par pas."""
    odom = o.Odometry(gains=gains or o.Gains())
    for v, ww, wg in steps:
        odom.step(dt, v, ww, wg)
    return odom


def test_ligne_droite():
    odom = run([(0.2, 0.0, 0.0)] * 250)     # 5 s a 0.2 m/s
    assert odom.x == pytest.approx(1.0, abs=1e-9)
    assert odom.y == pytest.approx(0.0, abs=1e-12)
    assert odom.yaw == pytest.approx(0.0, abs=1e-12)
    assert odom.source == o.GYRO and odom.fallbacks == 0


def test_le_cap_suit_le_gyro_pas_les_roues():
    """Roues qui patinent : elles annoncent une rotation, le gyromètre non."""
    odom = run([(0.0, 1.0, 0.0)] * 100)     # 2 s de patinage pur
    assert odom.yaw == pytest.approx(0.0, abs=1e-12)
    # et l'inverse : le robot tourne, les roues ne le voient pas
    odom = run([(0.0, 0.0, 1.0)] * 100)
    assert math.degrees(odom.yaw) == pytest.approx(math.degrees(2.0), abs=1e-9)


def test_cercle_ferme():
    """Un tour complet a vitesse constante ramene au point de depart."""
    v, w = 0.15, 0.5
    steps = int(round(2 * math.pi / w / DT))
    odom = run([(v, w, w)] * steps)
    assert math.hypot(odom.x, odom.y) < 0.003        # 3 mm sur 1.9 m parcourus
    # Le tour ne tombe pas sur un nombre entier de pas : 0.2 deg de residu.
    assert abs(math.degrees(odom.yaw)) < 0.25


def test_repli_sur_les_roues_sans_gyro():
    odom = run([(0.1, 0.3, None)] * 50)
    assert odom.source == o.WHEELS and odom.fallbacks == 50
    assert odom.yaw == pytest.approx(0.3, abs=1e-9)


def test_trame_de_zeros_rejetee():
    assert not o.usable((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))   # panne I2C
    assert o.usable((0.0, 0.0, 0.0), (0.0, 0.0, 9.81))      # robot vraiment a l'arret
    assert o.usable((0.0, 0.0, 0.4), (0.0, 0.0, 9.81))
    assert not o.usable(None, (0.0, 0.0, 9.81))


def test_biais_estime_a_l_arret_puis_retire():
    biais = math.radians(0.5)
    odom = run([(0.0, 0.0, biais)] * 500)            # 10 s a l'arret
    assert odom.bias == pytest.approx(biais, rel=0.02)
    # La derive du debut, le temps que l'estimation converge, ne se rattrape
    # pas ; ce qui compte est qu'elle s'arrete ensuite.
    apres_convergence = odom.yaw
    for _ in range(500):
        odom.step(DT, 0.0, 0.0, biais)
    assert abs(math.degrees(odom.yaw - apres_convergence)) < 0.02   # 10 s de plus
    # En mouvement, le biais reste retire et n'est plus reestime.
    depart, avant = odom.bias, odom.yaw
    odom.step(DT, 0.2, 0.0, biais)
    assert odom.bias == depart
    assert odom.yaw == pytest.approx(avant, abs=1e-5)


def test_biais_borne():
    gains = o.Gains(bias_max=math.radians(0.2))
    odom = run([(0.0, 0.0, math.radians(0.5))] * 2000, gains=gains)
    assert odom.bias == pytest.approx(gains.bias_max)


def test_une_vraie_rotation_n_est_pas_prise_pour_un_biais():
    """Robot pousse a la main : les roues ne tournent pas, le gyromètre si."""
    odom = run([(0.0, 0.0, math.radians(20.0))] * 100)
    assert odom.bias == 0.0
    assert math.degrees(odom.yaw) == pytest.approx(40.0, abs=0.1)


def test_echelle_appliquee():
    gains = o.Gains(scale=1.02)
    odom = run([(0.0, 0.0, 1.0)] * 50, gains=gains)   # 1 s a 1 rad/s
    assert odom.yaw == pytest.approx(1.02, abs=1e-9)


def test_le_biais_ne_se_reestime_pas_en_mouvement():
    odom = run([(0.3, 0.0, math.radians(2.0))] * 200)
    assert odom.bias == 0.0
    assert math.degrees(odom.yaw) == pytest.approx(8.0, abs=0.1)
