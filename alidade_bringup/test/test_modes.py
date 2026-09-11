"""Plans de transition entre modes, sans ROS."""
import pytest

from alidade_bringup import modes as m


def steps(plan):
    return [(t, a) for t, a, _ in plan.steps]


def test_idle_to_mapping_starts_slam_fresh():
    p = m.plan(m.Context(current=m.IDLE), m.MAPPING)
    assert steps(p) == [('nav', 'cancel'), ('slam', 'fresh')]


def test_mapping_to_localization_swaps_sources():
    ctx = m.Context(current=m.MAPPING, maps=['salon'], slam_active=True)
    p = m.plan(ctx, m.LOCALIZATION, 'salon')
    assert steps(p) == [('nav', 'cancel'), ('slam', 'deactivate'), ('map_server', 'load'), ('amcl', 'activate')]
    assert p.map_name == 'salon'
    assert p.steps[2][2] == 'salon'


def test_localization_needs_a_known_map():
    with pytest.raises(m.Refused, match='aucune carte'):
        m.plan(m.Context(current=m.IDLE, maps=['salon']), m.LOCALIZATION)
    with pytest.raises(m.Refused, match='introuvable'):
        m.plan(m.Context(current=m.IDLE, maps=['salon']), m.LOCALIZATION, 'garage')


def test_localization_uses_current_map_by_default():
    ctx = m.Context(current=m.IDLE, maps=['salon'], current_map='salon')
    assert m.plan(ctx, m.LOCALIZATION).map_name == 'salon'


def test_same_mode_is_a_noop_unless_map_changes():
    ctx = m.Context(current=m.LOCALIZATION, maps=['a', 'b'], current_map='a', amcl_active=True)
    assert m.plan(ctx, m.LOCALIZATION, 'a').steps == []
    p = m.plan(ctx, m.LOCALIZATION, 'b')
    assert steps(p) == [('nav', 'cancel'), ('amcl', 'deactivate'), ('map_server', 'load'), ('amcl', 'activate')]
    assert m.plan(m.Context(current=m.MAPPING, slam_active=True), m.MAPPING).steps == []


def test_back_to_idle_drops_localization():
    ctx = m.Context(current=m.LOCALIZATION, maps=['a'], current_map='a', amcl_active=True)
    assert steps(m.plan(ctx, m.IDLE)) == [('nav', 'cancel'), ('amcl', 'deactivate')]
    ctx = m.Context(current=m.MAPPING, slam_active=True)
    assert steps(m.plan(ctx, m.IDLE)) == [('nav', 'cancel'), ('slam', 'deactivate')]


def test_teleop_preserves_localization():
    """Conduire a la main ne doit rien detruire : la carte en cours survit a un
    aller-retour par TELEOP, sinon l'exploration repart de zero."""
    ctx = m.Context(current=m.MAPPING, slam_active=True)
    assert steps(m.plan(ctx, m.TELEOP)) == [('nav', 'cancel')]

    driving = m.Context(current=m.TELEOP, slam_active=True)
    assert steps(m.plan(driving, m.MAPPING)) == [('nav', 'cancel')]
    assert steps(m.plan(driving, m.EXPLORATION)) == [('nav', 'cancel'), ('explore', 'start')]

    # Depuis l'exploration : on arrete l'explorateur, on garde la carte.
    exploring = m.Context(current=m.EXPLORATION, slam_active=True)
    assert steps(m.plan(exploring, m.TELEOP)) == [('explore', 'stop'), ('nav', 'cancel')]

    # Idem avec amcl.
    localized = m.Context(current=m.LOCALIZATION, maps=['a'], current_map='a', amcl_active=True)
    assert steps(m.plan(localized, m.TELEOP)) == [('nav', 'cancel')]
    back = m.Context(current=m.TELEOP, maps=['a'], current_map='a', amcl_active=True)
    assert steps(m.plan(back, m.LOCALIZATION, 'a')) == [('nav', 'cancel')]

    # Mais TELEOP -> IDLE libere bien tout.
    assert steps(m.plan(driving, m.IDLE)) == [('nav', 'cancel'), ('slam', 'deactivate')]


def test_unknown_mode_is_refused():
    with pytest.raises(m.Refused, match='inconnu'):
        m.plan(m.Context(current=m.IDLE), 42)


def test_docking_needs_the_cube_in_view():
    with pytest.raises(m.Refused, match='cube non vu'):
        m.plan(m.Context(current=m.IDLE), m.DOCKING)
    p = m.plan(m.Context(current=m.IDLE, cube_visible=True), m.DOCKING)
    assert steps(p) == [('nav', 'cancel'), ('dock', 'start')]


def test_docking_preserves_localization():
    """S'approcher du cube ne suit que l'odometrie : la carte ou la
    localisation en cours survit, comme en TELEOP."""
    ctx = m.Context(current=m.LOCALIZATION, maps=['a'], current_map='a', amcl_active=True, cube_visible=True)
    assert steps(m.plan(ctx, m.DOCKING)) == [('nav', 'cancel'), ('dock', 'start')]
    back = m.Context(current=m.DOCKING, maps=['a'], current_map='a', amcl_active=True)
    assert steps(m.plan(back, m.LOCALIZATION, 'a')) == [('dock', 'cancel'), ('nav', 'cancel')]
    mapping = m.Context(current=m.DOCKING, slam_active=True)
    assert steps(m.plan(mapping, m.MAPPING)) == [('dock', 'cancel'), ('nav', 'cancel')]


def test_docking_again_restarts_the_approach_only_when_stopped():
    stopped = m.Context(current=m.DOCKING, cube_visible=True)
    assert steps(m.plan(stopped, m.DOCKING)) == [('dock', 'start')]
    running = m.Context(current=m.DOCKING, cube_visible=True, docking_active=True)
    assert m.plan(running, m.DOCKING).steps == []
    with pytest.raises(m.Refused, match='cube non vu'):
        m.plan(m.Context(current=m.DOCKING), m.DOCKING)


def test_leaving_docking_cancels_the_approach_first():
    ctx = m.Context(current=m.DOCKING, docking_active=True)
    assert steps(m.plan(ctx, m.IDLE)) == [('dock', 'cancel'), ('nav', 'cancel')]
    assert steps(m.plan(ctx, m.TELEOP)) == [('dock', 'cancel'), ('nav', 'cancel')]


def test_exploration_extends_mapping():
    # Depuis MAPPING : on garde la carte en cours, on lance l'explorateur.
    p = m.plan(m.Context(current=m.MAPPING, slam_active=True), m.EXPLORATION)
    assert steps(p) == [('nav', 'cancel'), ('explore', 'start')]
    # Depuis IDLE : carte neuve d'abord.
    p = m.plan(m.Context(current=m.IDLE), m.EXPLORATION)
    assert steps(p) == [('nav', 'cancel'), ('slam', 'fresh'), ('explore', 'start')]
    # En sortir arrete l'explorateur avant tout, et garde le SLAM pour MAPPING.
    ctx = m.Context(current=m.EXPLORATION, slam_active=True)
    assert steps(m.plan(ctx, m.MAPPING)) == [('explore', 'stop'), ('nav', 'cancel')]
    assert steps(m.plan(ctx, m.IDLE)) == [('explore', 'stop'), ('nav', 'cancel'), ('slam', 'deactivate')]
    assert m.nav_wanted(m.EXPLORATION, True)


def test_nav_only_when_localized_in_a_navigating_mode():
    assert m.nav_wanted(m.MAPPING, True)
    assert not m.nav_wanted(m.MAPPING, False)
    assert not m.nav_wanted(m.IDLE, True)
    assert not m.nav_wanted(m.TELEOP, True)
    assert m.nav_wanted(m.LOCALIZATION, True)
    # Le docking a sa propre loi de commande : pas de planificateur en concurrence.
    assert not m.nav_wanted(m.DOCKING, True)
