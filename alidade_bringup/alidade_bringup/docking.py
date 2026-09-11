"""Commande du docking sur le cube, sans ROS.

Deux temps, tous deux exprimes dans le repere odom :

  1. MISE EN PLACE, a l'odometrie : pivot, ligne droite, pivot, jusqu'a une
     pose dans l'axe de la face visee, a `staging` en retrait de la pose
     d'arret. Le robot recule si c'est le plus court. On la saute quand il y
     est deja a peu pres (aligne, et assez loin pour une approche).
  2. APPROCHE, asservie sur la vision : le robot suit l'axe de la face,
     corrige son ecart lateral, et s'arrete en atteignant la LIGNE d'arret.
     Pas un rayon autour d'un point : il ne peut pas la depasser en cherchant
     a l'atteindre, et le cube, qu'on perd a moins de 8 cm, reste dans le champ.

Pourquoi pas la loi du serveur de docking de nav2 (graceful controller) :
sans pivot sur place, et visant 25 cm au-dela de la pose d'arret, elle ne
rattrape pas un depart de travers sur 10 a 30 cm. Rejouee hors ligne sur des
departs a +-25 degres et +-5 cm, elle arrivait bien a 2 cm du point, mais
jusqu'a 27 degres de travers ; en simulation, le robot passait a cote du
point, continuait et perdait le cube.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float

    def ahead(self, d: float) -> 'Pose2D':
        """Meme cap, `d` metres devant (derriere si d < 0)."""
        return Pose2D(self.x + d * math.cos(self.yaw), self.y + d * math.sin(self.yaw), self.yaw)


@dataclass
class Gains:
    staging: float = 0.20        # m, retrait de la pose de mise en place sur la pose d'arret
    v_max: float = 0.08          # m/s
    v_min: float = 0.02          # m/s, en dessous les moteurs calent
    w_max: float = 0.8           # rad/s
    w_min: float = 0.25          # rad/s, pour que les pivots aboutissent
    k_v: float = 1.0             # 1/s, ralentissement a l'approche d'un point
    k_yaw: float = 2.0           # 1/s, correction de cap
    k_lat: float = 10.0          # 1/m, ecart lateral -> cap de rattrapage
    yaw_tol: float = math.radians(2.0)    # fin d'un pivot
    pos_tol: float = 0.01        # m, fin d'une ligne droite
    lat_tol: float = 0.015       # m, arrivee acceptee
    arrive_yaw_tol: float = math.radians(5.0)
    skip_lat: float = 0.015      # m, mise en place inutile en deca...
    skip_yaw: float = math.radians(6.0)   # ... et de ce cap...
    skip_min_run: float = 0.08   # m, ... s'il reste au moins cette course


def errors(robot: Pose2D, dock: Pose2D) -> tuple[float, float, float]:
    """(le long de l'axe, lateral, cap) du robot dans le repere de la pose
    d'arret. Le long de l'axe : negatif tant que la ligne n'est pas atteinte."""
    dx, dy = robot.x - dock.x, robot.y - dock.y
    c, s = math.cos(dock.yaw), math.sin(dock.yaw)
    return dx * c + dy * s, -dx * s + dy * c, wrap(robot.yaw - dock.yaw)


def staging_needed(robot: Pose2D, dock: Pose2D, g: Gains) -> bool:
    along, lat, yaw = errors(robot, dock)
    return not (abs(lat) < g.skip_lat and abs(yaw) < g.skip_yaw and along < -g.skip_min_run)


def arrived_ok(robot: Pose2D, dock: Pose2D, g: Gains) -> bool:
    _, lat, yaw = errors(robot, dock)
    return abs(lat) <= g.lat_tol and abs(yaw) <= g.arrive_yaw_tol


def _turn(err: float, g: Gains) -> float:
    w = g.k_yaw * err
    return math.copysign(clamp(abs(w), g.w_min, g.w_max), w)


class Stager:
    """Pivot - ligne droite - pivot vers `target`, a l'odometrie."""

    def __init__(self, target: Pose2D, g: Gains):
        self.target, self.g = target, g
        self.phase = 'turn'
        self.backward = False

    def step(self, robot: Pose2D) -> tuple[float, float, bool]:
        g, t = self.g, self.target
        dx, dy = t.x - robot.x, t.y - robot.y
        dist = math.hypot(dx, dy)
        if self.phase == 'turn':
            if dist < g.pos_tol:
                self.phase = 'align'
            else:
                bearing = math.atan2(dy, dx)
                # Reculer plutot que faire demi-tour : le cube reste devant.
                self.backward = abs(wrap(bearing - robot.yaw)) > math.pi / 2
                heading = wrap(bearing + math.pi) if self.backward else bearing
                err = wrap(heading - robot.yaw)
                if abs(err) > g.yaw_tol:
                    return 0.0, _turn(err, g), False
                self.phase = 'drive'
        if self.phase == 'drive':
            direction = -1.0 if self.backward else 1.0
            travel = direction * (dx * math.cos(robot.yaw) + dy * math.sin(robot.yaw))
            if dist < g.pos_tol or travel <= 0.0:
                self.phase = 'align'
            else:
                bearing = math.atan2(dy, dx)
                heading = wrap(bearing + math.pi) if self.backward else bearing
                v = clamp(g.k_v * travel, g.v_min, g.v_max)
                w = clamp(g.k_yaw * wrap(heading - robot.yaw), -g.w_max, g.w_max)
                return direction * v, w, False
        err = wrap(t.yaw - robot.yaw)
        if abs(err) > g.yaw_tol:
            return 0.0, _turn(err, g), False
        return 0.0, 0.0, True


def approach(robot: Pose2D, dock: Pose2D, g: Gains) -> tuple[float, float, bool]:
    """Commande vers la ligne d'arret ; True une fois la ligne atteinte."""
    along, lat, yaw = errors(robot, dock)
    if along >= 0.0:
        return 0.0, 0.0, True
    v = clamp(g.k_v * -along, g.v_min, g.v_max)
    # Cap vise : rattraper l'axe d'autant plus franchement que l'ecart est grand.
    wanted = -math.atan(g.k_lat * lat)
    w = clamp(g.k_yaw * wrap(wanted - yaw), -g.w_max, g.w_max)
    return v, w, False
