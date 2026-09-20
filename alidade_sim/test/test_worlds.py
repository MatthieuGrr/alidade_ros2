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
