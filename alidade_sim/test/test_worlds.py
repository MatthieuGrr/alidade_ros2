"""Les mondes de simulation sont-ils bien formes ?

Un SDF mal forme ne fait pas echouer le build : Gazebo demarre, le monde est
vide ou incomplet, et on cherche ailleurs. Ce test-la coute une seconde.

Il a deja servi : un commentaire XML contenant « -- » rend le fichier invalide,
et la seule trace cote Gazebo est une scene a moitie construite.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

WORLDS = Path(__file__).resolve().parent.parent / 'worlds'


@pytest.mark.parametrize('name', sorted(p.name for p in WORLDS.glob('*.sdf')))
def test_monde_bien_forme(name):
    root = ET.parse(WORLDS / name).getroot()
    assert root.tag == 'sdf'
    world = root.find('world')
    assert world is not None, 'pas de <world>'
    plugins = {p.get('name') for p in world.findall('plugin')}
    # Sans ces deux systemes, le gpu_lidar et l'IMU restent muets SANS message.
    assert 'gz::sim::systems::Sensors' in plugins
    assert 'gz::sim::systems::Imu' in plugins


def test_le_refend_degage_l_axe_du_robot():
    """Le robot apparait en (0, 0) cap 0 : rien ne doit barrer son axe de pres.

    A sa place d'origine (y = 1.2) le refend barrait x = 0.5 a 0.7 juste devant
    le point d'apparition, et tout objet pose au-dela de 30 cm se retrouvait
    dedans ou derriere. Symptome : detection parfaite jusqu'a 30 cm puis plus
    rien, qu'on prend volontiers pour un defaut de rendu ou de texture.
    """
    world = ET.parse(WORLDS / 'arena.sdf').getroot().find('world')
    for model in world.iter('model'):
        for link in model.iter('link'):
            if link.get('name') != 'divider':
                continue
            _, y, _, _, _, _ = [float(v) for v in link.find('pose').text.split()]
            _, profondeur, _ = [float(v) for v in
                                link.find('.//collision/geometry/box/size').text.split()]
            assert y - profondeur / 2 >= 0.35, (
                'le refend redescend dans l axe du robot : '
                f'il commence a y = {y - profondeur / 2:.2f} m')
            return
    raise AssertionError('refend « divider » absent de arena.sdf')
