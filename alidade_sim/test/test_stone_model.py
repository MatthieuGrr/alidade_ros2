"""Le modele Gazebo de la pierre est-il toujours celui de config/stone.yaml ?

Les fichiers de models/stone/ sont ENGENDRES par scripts/make_stone.py et
commites : Gazebo n'a donc besoin ni d'OpenCV ni d'un build. Le risque est
qu'une cote change dans stone.yaml sans qu'on regenere -- la simulation
mentirait alors sans rien dire. Ce test est la pour que ce silence coute un
echec de build.
"""
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
MODEL = PKG / 'models' / 'stone'


@pytest.fixture(scope='module')
def spec():
    from alidade_bringup import params, stone
    return stone.load(params.bringup_share() / 'config' / 'stone.yaml')


@pytest.fixture(scope='module')
def sdf():
    return ET.fromstring((MODEL / 'model.sdf').read_text())


def test_les_fichiers_engendres_sont_la():
    for name in ('model.sdf', 'model.config', 'meshes/stone.obj', 'meshes/stone.mtl',
                 'materials/textures/stone.png'):
        assert (MODEL / name).exists(), f'{name} manquant : lancer make_stone.py'


def test_cotes_et_masse_suivent_stone_yaml(spec, sdf):
    box = sdf.find('.//collision/geometry/box/size').text.split()
    assert [float(v) for v in box] == pytest.approx(list(spec.size)), \
        'cotes du modele != stone.yaml : relancer make_stone.py'
    assert float(sdf.find('.//inertial/mass').text) == pytest.approx(spec.mass)


def test_inertie_de_boite_homogene(spec, sdf):
    x, y, z = spec.size
    m = spec.mass
    for key, value in (('ixx', y * y + z * z), ('iyy', x * x + z * z), ('izz', x * x + y * y)):
        assert float(sdf.find(f'.//inertia/{key}').text) == pytest.approx(m / 12 * value, rel=1e-4)


def test_le_maillage_a_bien_six_faces_orientees_vers_l_exterieur(spec):
    """Un maillage retourne donnerait des marqueurs en miroir, indecodables, et
    la panne serait muette : le detecteur ne verrait simplement rien."""
    obj = (MODEL / 'meshes' / 'stone.obj').read_text()
    verts = [[float(v) for v in l.split()[1:]] for l in obj.splitlines() if l.startswith('v ')]
    normals = [[float(v) for v in l.split()[1:]] for l in obj.splitlines() if l.startswith('vn ')]
    assert len(verts) == 24 and len(normals) == 6
    # Chaque quad est plaque sur sa face, et sa normale sort du volume.
    for face, normal in enumerate(normals):
        quad = verts[4 * face:4 * face + 4]
        centre = [sum(p[i] for p in quad) / 4 for i in range(3)]
        assert sum(c * n for c, n in zip(centre, normal)) > 0, f'face {face} retournee'
        axis = normal.index(max(normal, key=abs))
        assert abs(centre[axis]) == pytest.approx(spec.size[axis] / 2), f'face {face} mal placee'


def test_le_marqueur_est_sur_les_quatre_grandes_faces(spec):
    """Les 4 faces 320x110 partagent la region texturee du marqueur ; les deux
    petites (110x110) tombent sur la plage de carton uni."""
    obj = (MODEL / 'meshes' / 'stone.obj').read_text()
    uv = [[float(v) for v in l.split()[1:]] for l in obj.splitlines() if l.startswith('vt ')]
    big = [i for i in range(6) if max(u for u, _ in uv[4 * i:4 * i + 4]) > 0.5]
    assert len(big) == 4, 'le marqueur doit couvrir les 4 grandes faces'
    normals = [[float(v) for v in l.split()[1:]] for l in obj.splitlines() if l.startswith('vn ')]
    # Les grandes faces sont celles dont la normale n'est pas selon le grand axe.
    assert all(abs(normals[i][0]) < 0.5 for i in big)


def test_la_texture_porte_le_bon_identifiant(spec):
    cv2 = pytest.importorskip('cv2')
    img = cv2.imread(str(MODEL / 'materials' / 'textures' / 'stone.png'), cv2.IMREAD_GRAYSCALE)
    assert img is not None
    dic = (cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f'DICT_{spec.marker_dict}'))
           if hasattr(cv2.aruco, 'getPredefinedDictionary') else
           cv2.aruco.Dictionary_get(getattr(cv2.aruco, f'DICT_{spec.marker_dict}')))
    p = (cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create')
         else cv2.aruco.DetectorParameters())
    p.minMarkerDistanceRate = 0.03      # voir alidade_bringup/config/aruco.yaml
    ids = cv2.aruco.detectMarkers(img, dic, parameters=p)[1]
    assert ids is not None and ids.flatten().tolist() == [spec.marker_id]
