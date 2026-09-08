"""Machine d'etats des modes du robot, sans ROS.

Le gestionnaire (scripts/mode_manager.py) demande ici un plan : la liste des
etapes a executer pour passer d'un mode a un autre, ou le motif du refus. Les
etapes sont des paires (cible, action) que le noeud sait executer ; les
tester ne demande ni robot ni simulation.

Regles :
  - une seule source de map -> odom a la fois : slam_toolbox (MAPPING,
    EXPLORATION) ou amcl (LOCALIZATION, DOCKING) ;
  - LOCALIZATION exige une carte connue ;
  - nav2 n'est pas pilote ici : le gestionnaire le demarre des que map -> odom
    existe dans un mode qui navigue, et le met en pause sinon
    (voir mode_manager.py, reconciliation) ;
  - EXPLORATION et DOCKING sont reserves : phases 5 et 6 de la feuille de route.
"""
from __future__ import annotations

from dataclasses import dataclass, field

IDLE, TELEOP, MAPPING, LOCALIZATION, EXPLORATION, DOCKING = range(6)

NAMES = {IDLE: 'IDLE', TELEOP: 'TELEOP', MAPPING: 'MAPPING', LOCALIZATION: 'LOCALIZATION',
         EXPLORATION: 'EXPLORATION', DOCKING: 'DOCKING'}
BY_NAME = {v: k for k, v in NAMES.items()}

SLAM_MODES = {MAPPING, EXPLORATION}
AMCL_MODES = {LOCALIZATION, DOCKING}
NAVIGATING_MODES = SLAM_MODES | AMCL_MODES
IMPLEMENTED = {IDLE, TELEOP, MAPPING, LOCALIZATION}


@dataclass
class Context:
    """Ce que le gestionnaire sait au moment de la demande."""
    current: int
    maps: list[str] = field(default_factory=list)
    current_map: str = ''
    slam_active: bool = False
    amcl_active: bool = False


@dataclass
class Plan:
    target: int
    steps: list[tuple] = field(default_factory=list)
    map_name: str = ''

    @property
    def summary(self) -> str:
        return ' ; '.join(f'{target} {action}' + (f' {arg}' if arg else '')
                          for target, action, arg in self.steps)


class Refused(Exception):
    pass


def name_of(mode: int) -> str:
    return NAMES.get(mode, f'?{mode}')


def plan(ctx: Context, target: int, map_name: str = '') -> Plan:
    """Etapes pour aller vers `target`, ou Refused avec un motif lisible."""
    if target not in NAMES:
        raise Refused(f'mode inconnu : {target}')
    if target not in IMPLEMENTED:
        raise Refused(f'{name_of(target)} : pas encore disponible (feuille de route)')

    chosen_map = map_name or ctx.current_map
    if target in AMCL_MODES:
        if not chosen_map:
            raise Refused('LOCALIZATION : aucune carte choisie')
        if chosen_map not in ctx.maps:
            raise Refused(f"LOCALIZATION : carte '{chosen_map}' introuvable "
                          f"(disponibles : {', '.join(ctx.maps) or 'aucune'})")

    steps: list[tuple] = []
    same_map = target in AMCL_MODES and ctx.amcl_active and chosen_map == ctx.current_map
    if target == ctx.current and (target not in AMCL_MODES or same_map):
        return Plan(target, [], chosen_map if target in AMCL_MODES else ctx.current_map)

    # Tout changement commence par lacher le but en cours.
    steps.append(('nav', 'cancel', ''))

    # Quitter la source de localisation qui ne sert plus.
    if ctx.slam_active and target not in SLAM_MODES:
        steps.append(('slam', 'deactivate', ''))
    if ctx.amcl_active and (target not in AMCL_MODES or not same_map):
        steps.append(('amcl', 'deactivate', ''))

    # Entrer dans la nouvelle.
    if target in SLAM_MODES and not ctx.slam_active:
        steps.append(('slam', 'fresh', ''))
    if target in AMCL_MODES and not same_map:
        steps.append(('map_server', 'load', chosen_map))
        steps.append(('amcl', 'activate', ''))

    return Plan(target, steps, chosen_map if target in AMCL_MODES else '')


def nav_wanted(mode: int, localized: bool) -> bool:
    """nav2 doit tourner : mode qui navigue ET repere map disponible."""
    return mode in NAVIGATING_MODES and localized
