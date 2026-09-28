#!/usr/bin/env python3
"""Pose et retire des pierres dans la simulation en cours.

    ros2 run alidade_sim stones.py spawn 0.6,0,90              # une pierre couchee
    ros2 run alidade_sim stones.py spawn 0.6,0,90 0.6,0,90,2   # une pile de deux
    ros2 run alidade_sim stones.py spawn 0.5,-0.1,70 0.8,0.1,110
    ros2 run alidade_sim stones.py clear                       # retire tout

Une pierre est decrite comme partout ailleurs (aruco_bench.py compris) :
`x,y,cap[,niveau]`, le cap etant celui du GRAND AXE en degres -- 90 met donc
une grande face en face d'un robot pose a l'origine et tourne vers +x -- et le
niveau valant 1 pour celle du bas d'une pile. La hauteur se deduit de
config/stone.yaml.

Les poses sont celles du MONDE. Tant que le robot simule n'a pas glisse, odom
vaut le monde, et une pierre posee en (0.6, 0) est bien a 60 cm devant lui.

Seules les pierres COUCHEES sont posables : c'est le perimetre du docking, et
une pierre debout ne serait de toute facon pas abordable (marqueur a 160 mm,
hors de l'image en deca d'un metre, et 320 mm de haut que le LiDAR voit, donc
arret du moniteur de collision a 22 cm). Les ajouter demanderait un tangage de
90 degres ici et rien d'autre.

Passe par les services gz plutot que par `ros_gz_sim create` : celui-ci ne sait
que poser, et une campagne d'essais doit surtout savoir retirer.
"""
from __future__ import annotations

import argparse
import math
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory

from alidade_bringup import params, stone

MAX_STONES = 40         # borne du balayage de `clear` : au-dela, ce n'est plus une scene


def gz(service: str, reqtype: str, req: str, timeout_ms: int = 25000,
       tries: int = 2) -> tuple[bool, str]:
    """Appel de service gz, avec un delai GENEREUX et une seconde chance.

    Le serveur repond en quelques millisecondes au repos, mais le PREMIER
    `create` d'une session charge le maillage et peut depasser 15 s ; une
    simulation chargee (rendu logiciel, physique, une trentaine de noeuds)
    allonge encore. Mesure : 10 s echouent, 20 s passent.
    """
    for attempt in range(tries):
        out = subprocess.run(
            ['gz', 'service', '-s', service, '--reqtype', reqtype,
             '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout_ms), '--req', req],
            capture_output=True, text=True)
        if 'data: true' in out.stdout:
            return True, out.stdout.strip()
        text = (out.stdout + out.stderr).strip()
        if 'timed out' not in text or attempt == tries - 1:
            return False, text
    return False, text


def monde_en_cours() -> str:
    """Nom du monde que la simulation fait tourner, lu sur les sujets gz.

    A NE PAS remplacer par une valeur par defaut en dur : un appel de service
    sur un monde qui n'existe pas n'echoue pas, il EXPIRE. Le message parle
    alors de delai et on cherche une lenteur pendant que la table reste vide --
    trois campagnes ont ete perdues comme cela.
    """
    out = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True,
                         timeout=10).stdout
    noms = {l.split('/')[2] for l in out.splitlines() if l.startswith('/world/')}
    if len(noms) != 1:
        raise SystemExit(f"monde indeterminable ({sorted(noms) or 'aucun'}) : "
                         "precise --world")
    return noms.pop()


def parse(text: str):
    parts = text.split(',')
    if not 3 <= len(parts) <= 4:
        raise argparse.ArgumentTypeError(f"'{text}' : attendu x,y,cap[,niveau]")
    x, y, yaw = (float(v) for v in parts[:3])
    return x, y, yaw, int(parts[3]) if len(parts) == 4 else 1


def cmd_spawn(args):
    spec = stone.load(params.bringup_share() / 'config' / 'stone.yaml')
    model = f"{get_package_share_directory('alidade_sim')}/models/stone/model.sdf"
    poses = 0
    for i, (x, y, yaw, level) in enumerate(args.stones, start=1):
        z = spec.stacked(level)     # centre de la boite = centre du marqueur lateral
        a = math.radians(yaw)
        req = (f'sdf_filename: "{model}", name: "{args.prefix}{i}", allow_renaming: false, '
               f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{z: {math.sin(a / 2)}, w: {math.cos(a / 2)}}}}}')
        ok, text = gz(f'/world/{args.world}/create', 'gz.msgs.EntityFactory', req)
        poses += ok
        print(f'  {args.prefix}{i} : ({x:+.2f}, {y:+.2f}) cap {yaw:+.0f} deg, '
              f'niveau {level} (centre a {z * 1000:.0f} mm) -> '
              f'{"pose" if ok else "ECHEC : " + text}')
    # On ne s'arrete PAS au premier echec : une campagne qui perd une pierre sur
    # douze reste exploitable, alors qu'un abandon laisse la table VIDE sans
    # que rien ne le signale -- trois essais ont ete mesures comme cela.
    print(f'  {poses}/{len(args.stones)} pierres posees')
    return 0 if poses else 1


def cmd_clear(args):
    """Retire les pierres posees, en s'arretant des que la serie est finie.

    Balayer les 40 noms coute 20 s de delais d'attente, et une campagne
    d'essais appelle `clear` entre chaque scene : on s'arrete apres trois
    absences de suite, ce qui laisse passer un trou sans tout balayer.
    """
    removed, missing = 0, 0
    for i in range(1, MAX_STONES + 1):
        ok, _ = gz(f'/world/{args.world}/remove', 'gz.msgs.Entity',
                   f'name: "{args.prefix}{i}", type: MODEL', timeout_ms=1500, tries=1)
        removed, missing = removed + ok, 0 if ok else missing + 1
        if missing >= 3:
            break
    print(f'  {removed} pierre(s) retiree(s)')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default=None,
                    help='par defaut, le monde que la simulation fait tourner')
    ap.add_argument('--prefix', default='stone_')
    sub = ap.add_subparsers(dest='command', required=True)
    s = sub.add_parser('spawn', help='pose des pierres')
    s.add_argument('stones', nargs='+', type=parse)
    s.set_defaults(func=cmd_spawn)
    c = sub.add_parser('clear', help='retire toutes les pierres')
    c.set_defaults(func=cmd_clear)
    args = ap.parse_args()
    args.world = args.world or monde_en_cours()
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
