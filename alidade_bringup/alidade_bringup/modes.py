"""Machine d'etats des modes du robot, sans ROS.

Le gestionnaire (scripts/mode_manager.py) demande ici un plan : la liste des
etapes a executer pour passer d'un mode a un autre, ou le motif du refus. Les
etapes sont des paires (cible, action) que le noeud sait executer ; les
tester ne demande ni robot ni simulation.

Regles :
  - une seule source de map -> odom a la fois : slam_toolbox (MAPPING,
    EXPLORATION) ou amcl (LOCALIZATION) ;
  - LOCALIZATION exige une carte connue ;
  - la PLANIFICATION nav2 n'est pas pilotee ici : le gestionnaire la demarre
    des que map -> odom existe dans un mode qui navigue, et la met en pause
    sinon (voir mode_manager.py, reconciliation). La chaine de securite
    (lissage et moniteur de collision), elle, tourne toujours : elle protege
    aussi le pilotage manuel ;
  - TELEOP ne touche pas a la localisation : on peut conduire a la main au
    milieu d'une cartographie et y revenir sans perdre la carte ;
  - EXPLORATION = MAPPING plus l'explorateur de frontieres, qui pousse ses
    propres buts nav2 ; on l'arrete avant de quitter le mode ;
  - DOCKING approche le cube jusqu'a un ecart fixe (cube_docking.py).
    Il ne demarre que cube VU : pas de recherche, l'operateur amene le robot a
    portee. Il ne suit que l'odometrie et la pose du cube, et preserve donc la
    localisation en cours comme TELEOP. Redemander DOCKING une fois arrete
    relance une approche ; on annule l'approche avant de quitter le mode.
"""
from __future__ import annotations

from dataclasses import dataclass, field

IDLE, TELEOP, MAPPING, LOCALIZATION, EXPLORATION, DOCKING = range(6)

NAMES = {IDLE: 'IDLE', TELEOP: 'TELEOP', MAPPING: 'MAPPING', LOCALIZATION: 'LOCALIZATION',
         EXPLORATION: 'EXPLORATION', DOCKING: 'DOCKING'}
BY_NAME = {v: k for k, v in NAMES.items()}

SLAM_MODES = {MAPPING, EXPLORATION}
AMCL_MODES = {LOCALIZATION}
NAVIGATING_MODES = SLAM_MODES | AMCL_MODES
# Modes qui gardent la source de localisation en cours quelle qu'elle soit :
# conduire a la main pendant une cartographie est un usage normal, et revenir
# a MAPPING ou EXPLORATION doit reprendre la carte, pas repartir de zero.
PRESERVING_MODES = {TELEOP, DOCKING}
IMPLEMENTED = {IDLE, TELEOP, MAPPING, LOCALIZATION, EXPLORATION, DOCKING}


@dataclass
class Context:
    """Ce que le gestionnaire sait au moment de la demande."""
    current: int
    maps: list[str] = field(default_factory=list)
    current_map: str = ''
    slam_active: bool = False
    amcl_active: bool = False
    cube_visible: bool = False     # pose d'arret recue il y a moins d'une seconde
    docking_active: bool = False   # une approche est en cours


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

    if target == DOCKING:
        if ctx.current == DOCKING and ctx.docking_active:
            return Plan(target, [], ctx.current_map)
        if not ctx.cube_visible:
            raise Refused('DOCKING : cube non vu (a amener dans le champ, de 8 a 42 cm devant le robot)')
        if ctx.current == DOCKING:
            return Plan(target, [('dock', 'start', '')], ctx.current_map)

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

    # L'exploration s'arrete avant tout le reste : elle tient un but nav2. De
    # meme l'approche du cube, qui commande les roues.
    if ctx.current == EXPLORATION:
        steps.append(('explore', 'stop', ''))
    if ctx.current == DOCKING:
        steps.append(('dock', 'cancel', ''))

    # Tout changement commence par lacher le but en cours.
    steps.append(('nav', 'cancel', ''))

    # Quitter la source de localisation qui ne sert plus -- sauf vers un mode
    # qui la preserve (TELEOP, DOCKING).
    keep = target in PRESERVING_MODES
    if ctx.slam_active and not (target in SLAM_MODES or keep):
        steps.append(('slam', 'deactivate', ''))
    if ctx.amcl_active and not ((target in AMCL_MODES and same_map) or keep):
        steps.append(('amcl', 'deactivate', ''))

    # Entrer dans la nouvelle. L'exploration continue la carte en cours si le
    # SLAM tourne deja (MAPPING -> EXPLORATION), sinon en commence une.
    if target in SLAM_MODES and not ctx.slam_active:
        steps.append(('slam', 'fresh', ''))
    if target in AMCL_MODES and not same_map:
        steps.append(('map_server', 'load', chosen_map))
        steps.append(('amcl', 'activate', ''))
    if target == EXPLORATION:
        steps.append(('explore', 'start', ''))
    if target == DOCKING:
        steps.append(('dock', 'start', ''))

    return Plan(target, steps, chosen_map if target in AMCL_MODES else '')


def nav_wanted(mode: int, localized: bool) -> bool:
    """nav2 doit tourner : mode qui navigue ET repere map disponible."""
    return mode in NAVIGATING_MODES and localized
