"""Coherence de la description du cube cible (config/cube.yaml).

Ce fichier se relit et se retape a la main : une faute de signe y serait
invisible et fausserait toutes les poses en aval. On verifie donc que chaque
face definit bien un repere orthonorme DIRECT, et que les six faces forment
bien un cube.
"""
import itertools

import numpy as np
import pytest
import yaml

from alidade_bringup import params


@pytest.fixture(scope='module')
def cube():
    return yaml.safe_load((params.bringup_share() / 'config' / 'cube.yaml').read_text())


def rotation(face):
    """Axes du marqueur dans le repere du cube : x = up ^ normal, y = up, z = normal."""
    z = np.array(face['normal'], float)
    y = np.array(face['up'], float)
    return np.column_stack([np.cross(y, z), y, z])


def test_six_faces_numerotees_de_1_a_6(cube):
    assert sorted(f['id'] for f in cube['faces']) == [1, 2, 3, 4, 5, 6]
    assert cube['marker_size'] < cube['edge'], 'le marqueur doit tenir sur la face'


def test_chaque_face_donne_un_repere_orthonorme_direct(cube):
    for face in cube['faces']:
        z, y = np.array(face['normal'], float), np.array(face['up'], float)
        assert np.isclose(np.linalg.norm(z), 1.0), f"{face['nom']} : normale non unitaire"
        assert np.isclose(np.linalg.norm(y), 1.0), f"{face['nom']} : 'up' non unitaire"
        assert np.isclose(z @ y, 0.0), f"{face['nom']} : 'up' n'est pas dans le plan de la face"
        rot = rotation(face)
        assert np.allclose(rot.T @ rot, np.eye(3)), f"{face['nom']} : repere non orthonorme"
        assert np.isclose(np.linalg.det(rot), 1.0), f"{face['nom']} : repere indirect (miroir)"


def test_les_six_normales_couvrent_les_six_directions(cube):
    normales = [tuple(f['normal']) for f in cube['faces']]
    attendues = {(1., 0., 0.), (-1., 0., 0.), (0., 1., 0.),
                 (0., -1., 0.), (0., 0., 1.), (0., 0., -1.)}
    assert set(normales) == attendues, 'les faces ne forment pas un cube'


def test_les_faces_opposees_le_sont_vraiment(cube):
    for a, b in itertools.combinations(cube['faces'], 2):
        na, nb = np.array(a['normal']), np.array(b['normal'])
        if np.allclose(na, -nb):
            assert a['nom'] != b['nom']
            break
    else:
        pytest.fail('aucune paire de faces opposees')


def test_reperes_conformes_a_la_description_du_cube(cube):
    """La convention retenue : x vers l'avant (face 'devant'), y vers la gauche
    (face 'gauche'), z vers le haut (face 'haut')."""
    par_nom = {f['nom']: np.array(f['normal'], float) for f in cube['faces']}
    assert np.allclose(par_nom['devant'], [1, 0, 0])
    assert np.allclose(par_nom['gauche'], [0, 1, 0])
    assert np.allclose(par_nom['haut'], [0, 0, 1])
    # Les faces verticales ont leur motif droit. Le dessus a son motif vers la
    # gauche (mesure sur le cube reel), le dessous vers l'avant (tel que decrit,
    # pas encore verifie).
    par_id = {f['nom']: np.array(f['up'], float) for f in cube['faces']}
    for nom in ('devant', 'derriere', 'gauche', 'droite'):
        assert np.allclose(par_id[nom], [0, 0, 1]), f'{nom} : haut du motif attendu vers le haut'
    assert np.allclose(par_id['haut'], [0, 1, 0]), 'haut : motif attendu vers la gauche'
    assert np.allclose(par_id['bas'], [1, 0, 0]), 'bas : motif attendu vers l avant'
