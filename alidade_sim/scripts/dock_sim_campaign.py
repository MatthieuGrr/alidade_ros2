#!/usr/bin/env python3
"""Campagne de docking sur les pierres, en simulation, contre la verite terrain.

    ros2 launch alidade_sim sim.launch.py nav:=true hardware_revision:=v2 headless:=true
    ros2 run alidade_sim dock_sim_campaign.py                 # la batterie, une passe
    ros2 run alidade_sim dock_sim_campaign.py --repeats 5     # avec departs perturbes
    ros2 run alidade_sim dock_sim_campaign.py --only pile     # un scenario

La revision v2 n'est pas un detail : elle decrit le support de camera a 15
degres. Avec v1 (23.5 degres) le marqueur d'une pierre couchee n'est dans
l'image qu'entre 9 et 16 cm, et aucun docking ne peut aboutir.

Ce que mesure la campagne, et que le journal du robot ne dit pas : l'ecart
REEL entre la face avant du robot et la face de la pierre, lu dans Gazebo, et
le DEPLACEMENT de la pierre. Une pierre de 90 g qu'on pousse de deux
centimetres, c'est un echec meme si l'action rend un succes.

Le robot est remis a l'origine entre les essais (`set_pose`), la pierre est
posee devant lui : ce sont les POSES DES PIERRES qui changent d'un scenario a
l'autre, pas celle du robot. Un `set_pose` casse la continuite de l'odometrie,
qui n'est jamais absolue ici -- le docking ne travaille qu'en relatif.
"""
from __future__ import annotations

import argparse
import math
import random
import re
import statistics
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot
from rclpy.action import ActionClient
from rclpy.node import Node

from alidade_bringup import params, stone

SPEC = stone.load(params.bringup_share() / 'config' / 'stone.yaml')
# v2 : la campagne n'a de sens que sur cette revision, et seule l'empreinte est
# lue ici -- identique a v1, mais la lire ailleurs serait un mensonge de plus.
FRONT = params.load_revision('v2')['footprint']['front_x']

# (nom, pierres posees, pierre a aborder). Meme description qu'ailleurs :
# x, y, cap du grand axe en degres, niveau dans la pile.
def battery():
    return [
        ('seule 40 cm',   [(0.40, 0.00, 90, 1)], 0),
        ('seule 60 cm',   [(0.60, 0.00, 90, 1)], 0),
        ('de travers',    [(0.45, 0.00, 70, 1)], 0),
        ('decalee',       [(0.45, 0.10, 100, 1)], 0),
        ('pile de 2',     [(0.45, 0.00, 90, 1), (0.45, 0.00, 90, 2)], 0),
        ('pile de 3',     [(0.45, 0.00, 90, 1), (0.45, 0.00, 90, 2), (0.45, 0.00, 90, 3)], 0),
        ('deux pierres',  [(0.40, -0.04, 90, 1), (0.80, 0.06, 90, 1)], 0),
    ]


def gz_pose(name):
    """Verite terrain d'un modele : (x, y, cap). Meme lecture que measure_odom.py."""
    out = subprocess.run(['gz', 'model', '-m', name, '-p'],
                         capture_output=True, text=True, timeout=10).stdout
    m = re.search(r'Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\[([^\]]+)\]\s*\[([^\]]+)\]', out)
    if not m:
        return None
    xyz = [float(v) for v in m.group(1).split()]
    rpy = [float(v) for v in m.group(2).split()]
    return xyz[0], xyz[1], rpy[2]


def gz_set_pose(world, name, x, y, z, yaw):
    subprocess.run(
        ['gz', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'gz.msgs.Pose',
         '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req',
         f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
         f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}'],
        capture_output=True, text=True)


def stones(*args):
    return subprocess.run(['ros2', 'run', 'alidade_sim', 'stones.py'] + list(args),
                          capture_output=True, text=True)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def errors(robot, face_centre, normal_yaw):
    """(ecart le long de la normale, ecart lateral, ecart de cap) a l'arret.

    L'ecart est celui de la FACE AVANT du robot a la face de la pierre : c'est
    ce que `gap` regle, et ce qu'un operateur mesure au reglet.
    """
    n = (math.cos(normal_yaw), math.sin(normal_yaw))
    dx, dy = robot[0] - face_centre[0], robot[1] - face_centre[1]
    along = dx * n[0] + dy * n[1] - FRONT
    lateral = -dx * n[1] + dy * n[0]
    return along, lateral, wrap(robot[2] - (normal_yaw + math.pi))


class Campaign(Node):
    def __init__(self, world):
        super().__init__('dock_sim_campaign')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.world = world
        self.dock_pose = None
        self.seen_at = None
        self.create_subscription(PoseStamped, '/stone/dock_pose', self.on_dock, 10)
        self.client = ActionClient(self, DockRobot, '/dock_robot')

    def on_dock(self, msg):
        self.dock_pose, self.seen_at = msg, time.monotonic()

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_target(self, timeout=20.0):
        # Vider d'abord ce que DDS a mis en file pendant les appels de spawn,
        # ou le noeud ne tournait pas : une pose d'arret de la scene PRECEDENTE
        # ferait croire a une detection immediate, et la mesure porterait sur
        # la mauvaise pierre. Vu en premiere campagne.
        self.spin(0.5)
        self.dock_pose = None
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.dock_pose is not None:
                return True
        return False

    def dock(self, timeout=120.0):
        """Meme requete que le gestionnaire de modes : pose d'arret vue, pas de
        navigation vers la pose de mise en place."""
        if not self.client.wait_for_server(timeout_sec=10.0):
            return None, 'docking_server absent'
        pose = PoseStamped(pose=self.dock_pose.pose)
        pose.header.frame_id = self.dock_pose.header.frame_id
        goal = DockRobot.Goal(use_dock_id=False, dock_pose=pose, dock_type='stone',
                              navigate_to_staging_pose=False)
        send = self.client.send_goal_async(goal)
        end = time.monotonic() + 10.0
        while rclpy.ok() and not send.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        handle = send.result()
        if handle is None or not handle.accepted:
            return None, 'requete refusee'
        result = handle.get_result_async()
        end = time.monotonic() + timeout
        while rclpy.ok() and not result.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result.done():
            return None, 'delai depasse'
        res = result.result().result
        return res, ('succes' if res.success else f'echec {res.error_code} {res.error_msg}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='arena')
    ap.add_argument('--repeats', type=int, default=1)
    ap.add_argument('--only', help='ne garder que les scenarios dont le nom contient ceci')
    ap.add_argument('--gap', type=float, default=0.10, help='ecart vise, m')
    ap.add_argument('--seed', type=int, default=1)
    args = ap.parse_args()
    random.seed(args.seed)

    cases = [c for c in battery() if not args.only or args.only in c[0]]
    rclpy.init()
    node = Campaign(args.world)
    rows = []
    print(f"  {'scenario':16s} {'depart':>7} {'resultat':>22} {'ecart':>8} {'lateral':>8} "
          f"{'cap':>7} {'pierre':>8} {'duree':>7}")
    for repeat in range(args.repeats):
        for (name, placed, target) in cases:
            perturb = 0.0 if repeat == 0 else math.radians(random.uniform(-12, 12))
            stones('clear')
            gz_set_pose(args.world, 'alidade', 0.0, 0.0, 0.02, perturb)
            node.spin(2.0)
            out = stones('spawn', *[f'{x},{y},{yaw},{lvl}' for (x, y, yaw, lvl) in placed])
            if out.stdout.count('pose') != len(placed):
                print(f'  {name:16s} ECHEC du spawn : {out.stdout.strip()}')
                continue
            # Attendre que Gazebo ait vraiment instancie la pierre : le
            # service `create` repond avant que le modele soit interrogeable,
            # et `gz model -m` rend alors une reponse vide.
            spawned = None
            for _ in range(10):
                node.spin(1.0)
                spawned = gz_pose(f'stone_{target + 1}')
                if spawned is not None:
                    break
            if spawned is None:
                print(f'  {name:16s} pierre jamais apparue dans Gazebo')
                rows.append((name, perturb, 'pierre absente', None, None, None, None, 0.0))
                continue
            if not node.wait_target():
                rows.append((name, perturb, 'aucune cible', None, None, None, None, 0.0))
                print(f"  {name:16s} {math.degrees(perturb):+6.1f}d {'aucune cible':>22}")
                continue
            start = time.monotonic()
            _, verdict = node.dock()
            duration = time.monotonic() - start
            node.spin(1.0)
            robot, final = gz_pose('alidade'), gz_pose(f'stone_{target + 1}')
            if robot is None:
                print(f'  {name:16s} pose du robot illisible dans Gazebo')
                rows.append((name, perturb, 'verite terrain absente', None, None, None, None, 0.0))
                continue
            # Une pierre devenue illisible a ete poussee hors de la table, ou
            # le service a cale : dans les deux cas l'essai ne se mesure pas.
            moved = math.dist(final[:2], spawned[:2]) if final else float('inf')
            # Face abordee : des deux grandes faces, celle dont la normale
            # sortante regarde le robot.
            to_robot = math.atan2(robot[1] - spawned[1], robot[0] - spawned[0])
            best = min((spawned[2] + math.pi / 2, spawned[2] - math.pi / 2),
                       key=lambda a: abs(wrap(a - to_robot)))
            centre = (spawned[0] + SPEC.size[1] / 2 * math.cos(best),
                      spawned[1] + SPEC.size[1] / 2 * math.sin(best))
            along, lateral, heading = errors(robot, centre, best)
            rows.append((name, perturb, verdict, along, lateral, heading, moved, duration))
            print(f"  {name:16s} {math.degrees(perturb):+6.1f}d {verdict[:22]:>22} "
                  f"{along * 100:7.1f}cm {lateral * 1000:+7.1f}mm {math.degrees(heading):+6.1f}d "
                  f"{moved * 1000:7.1f}mm {duration:6.1f}s")
    stones('clear')

    # Un bilan qui melange « le docking a rate » et « la pierre n'etait pas dans
    # le champ au depart » ne dit rien d'utile : le second cas n'est meme pas un
    # essai de docking, l'action n'a jamais demarre. On les separe.
    mesures = [r for r in rows if r[3] is not None]
    sans_cible = [r for r in rows if r[2] == 'aucune cible']
    illisibles = [r for r in rows if r[2].startswith(('pierre absente', 'verite terrain'))]
    def dans_tolerance(r):
        return (abs(r[3] - args.gap) < 0.015 and abs(r[4]) < 0.015
                and abs(r[5]) < math.radians(5))
    bons = [r for r in mesures if dans_tolerance(r)]
    bouge = [r for r in mesures if r[6] > 0.005 and r[6] != float('inf')]
    print(f"\n  {len(rows)} essais : {len(mesures)} avec une cible acquise, "
          f"{len(sans_cible)} sans (le depart perturbe l'a sortie du champ), "
          f"{len(illisibles)} sans verite terrain")
    print(f"  {len(bons)}/{len(mesures)} dockings dans les tolerances "
          f"(ecart {args.gap * 100:.0f} +- 1.5 cm, lateral < 15 mm, cap < 5 deg)")
    if mesures:
        pousse = [r[6] for r in mesures if r[6] != float('inf')]
        print(f"  ecart {statistics.mean(r[3] for r in mesures) * 100:.1f} cm en moyenne "
              f"(extremes {min(r[3] for r in mesures) * 100:.1f} a "
              f"{max(r[3] for r in mesures) * 100:.1f}), "
              f"lateral {statistics.mean(abs(r[4]) for r in mesures) * 1000:.1f} mm, "
              f"cap {statistics.mean(abs(math.degrees(r[5])) for r in mesures):.1f} deg, "
              f"{statistics.mean(r[7] for r in mesures):.1f} s")
        print(f"  pierre deplacee au plus de {max(pousse) * 1000:.1f} mm "
              f"sur {len(pousse)} essais mesures ; {len(bouge)} au-dela de 5 mm")
    if sans_cible:
        print("  sans cible : " + ", ".join(
            f"{r[0]} (depart {math.degrees(r[1]):+.1f} deg)" for r in sans_cible))
    rclpy.shutdown()
    # Le code de retour ne juge que ce qui est mesurable.
    return 0 if mesures and len(bons) == len(mesures) and not bouge else 1


if __name__ == '__main__':
    sys.exit(main())
