"""Les mondes de simulation sont-ils bien formes, et la table conforme ?

Un SDF mal forme ne fait pas echouer le build : Gazebo demarre, le monde est
vide ou incomplet, et on cherche ailleurs. Ce test-la coute une seconde.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

WORLDS = Path(__file__).resolve().parent.parent / 'worlds'


def box(world, model, link):
    """Cotes de la boite de collision d'un link, en m."""
    for m in world.iter('model'):
        if m.get('name') != model:
            continue
        for l in m.iter('link'):
            if l.get('name') == link:
                return [float(v) for v in l.find('.//collision/geometry/box/size').text.split()]
    raise AssertionError(f'{model}/{link} absent')


@pytest.mark.parametrize('name', ['arena.sdf', 'eurobot.sdf'])
def test_monde_bien_forme(name):
    root = ET.parse(WORLDS / name).getroot()
    assert root.tag == 'sdf'
    world = root.find('world')
    assert world is not None, 'pas de <world>'
    plugins = {p.get('name') for p in world.findall('plugin')}
    # Sans ces deux systemes, le gpu_lidar et l'IMU restent muets SANS message.
    assert 'gz::sim::systems::Sensors' in plugins
    assert 'gz::sim::systems::Imu' in plugins


def test_table_aux_cotes_du_reglement():
    world = ET.parse(WORLDS / 'eurobot.sdf').getroot().find('world')
    x, y, _ = box(world, 'table', 'plateau')
    assert (x, y) == pytest.approx((3.0, 2.0)), 'plateau : 3000 x 2000 mm'
    for link in ('bordure_x0', 'bordure_x3000', 'bordure_y0', 'bordure_y2000'):
        dims = box(world, 'table', link)
        assert pytest.approx(0.070) == dims[2], f'{link} : bordure de 70 mm de haut'
        assert pytest.approx(0.022) == min(dims[:2]), f'{link} : 22 mm d epaisseur'


def test_les_bordures_passent_sous_le_lidar():
    """70 mm de bordure contre un plan de LiDAR a 153.7 mm : la table est
    invisible au LiDAR, et ce n'est pas un oubli. C'est pour cela qu'arena.sdf
    reste le monde des essais de cartographie."""
    import yaml
    from alidade_bringup import params
    laser_z = params.load_revision('v1')['frames']['expected']['laser'][2]
    world = ET.parse(WORLDS / 'eurobot.sdf').getroot().find('world')
    hauteur = box(world, 'table', 'bordure_y0')[2]
    assert hauteur < laser_z, 'si la bordure depassait le LiDAR, revoir le commentaire du monde'


# ------------------------------------------------------------------- balises
def cylindres(world, model):
    out = {}
    for m in world.iter('model'):
        if m.get('name') != model:
            continue
        for l in m.iter('link'):
            c = l.find('.//collision/geometry/cylinder')
            if c is not None:
                out[l.get('name')] = (float(c.find('radius').text), float(c.find('length').text),
                                      [float(v) for v in l.find('pose').text.split()][:3])
    return out


def test_trois_balises_aux_positions_du_reglement():
    """Supports a 94 mm du bord (100 mm de cote, retrait de 44 mm, tous deux
    ecrits au reglement general G.3.b) et repartis a y = 50, 1000 et 1950 mm,
    releves sur son plan."""
    world = ET.parse(WORLDS / 'eurobot.sdf').getroot().find('world')
    b = cylindres(world, 'balises')
    assert len(b) == 3, 'trois balises fixes par equipe'
    for rayon, longueur, (x, y, z) in b.values():
        assert rayon == pytest.approx(0.040), 'tuyau PVC de 80 mm'
        assert longueur <= 1.000 + 1e-9, 'le reglement plafonne la balise a 1000 mm'
        assert abs(x) == pytest.approx(0.094) or x == pytest.approx(3.094), \
            'la balise est hors du plateau, sur son support'
    assert sorted(round(p[2][1], 3) for p in b.values()) == [0.05, 1.0, 1.95]


def test_les_balises_traversent_le_plan_du_lidar():
    """Tout l'interet des balises : elles sont le seul relief qu'un LiDAR a
    153.7 mm rencontre sur une table dont les bordures font 70 mm. Une balise
    qui ne traverserait pas ce plan ne servirait a rien."""
    from alidade_bringup import params
    laser_z = params.load_revision('v1')['frames']['expected']['laser'][2]
    world = ET.parse(WORLDS / 'eurobot.sdf').getroot().find('world')
    for nom, (_, longueur, pose) in cylindres(world, 'balises').items():
        bas, haut = pose[2] - longueur / 2, pose[2] + longueur / 2
        assert bas <= laser_z <= haut, f'{nom} : hors du plan du LiDAR'


def test_les_six_supports_sont_hors_de_la_surface_de_jeu():
    world = ET.parse(WORLDS / 'eurobot.sdf').getroot().find('world')
    supports = [l for l in world.iter('link') if l.get('name', '').startswith('support_balise_')]
    assert len(supports) == 6, 'trois supports par equipe, deux equipes'
    for l in supports:
        x, y, _ = [float(v) for v in l.find('pose').text.split()][:3]
        taille = [float(v) for v in l.find('.//collision/geometry/box/size').text.split()]
        assert taille[0] == pytest.approx(0.1) and taille[1] == pytest.approx(0.1)
        assert x + taille[0] / 2 <= 0 or x - taille[0] / 2 >= 3.0, \
            'un support empiete sur la surface de jeu'
        assert 0 <= y <= 2.0
