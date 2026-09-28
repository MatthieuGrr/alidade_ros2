"""Recalage d'un scan LiDAR sur les bordures de la table, et son incertitude.

La table de la Coupe est un rectangle connu, borde sur ses quatre cotes par une
bordure de 70 mm. Avec le LiDAR abaisse sous cette hauteur (revision v3, plan de
scan a 45 mm), chaque tour de scan voit les bordures : de quoi retrouver la pose
du robot sans carte construite, sans SLAM et sans amers.

Ce module ne fait que la geometrie -- pas de ROS, pas de Gazebo -- pour que la
meme fonction serve au banc d'essai, aux tests unitaires et au robot.

TROIS CHOSES QU'IL FAUT SAVOIR AVANT DE S'EN SERVIR.

1. La table est SYMETRIQUE. Un rectangle est invariant par demi-tour autour de
   son centre : la pose (x, y, cap) et la pose (L-x, H-y, cap+180) produisent
   exactement le meme scan. Mesure faite sur 35 scans de simulation : le residu
   des deux solutions est identique au centieme de millimetre, avec le meme
   nombre d'inliers. Le scan ne porte pas « peu » d'information pour trancher,
   il n'en porte AUCUNE. D'ou `heading` : voir plus bas.

2. Le recalage rend une COVARIANCE, pas seulement une pose. Un robot qui fait
   face a un mur mesure parfaitement sa distance a ce mur et son cap, et pas du
   tout son glissement le long du mur. Publier la pose nue perd cette nuance ;
   publier la matrice normale inversee la donne a qui sait s'en servir (EKF,
   AMCL). Voir `Fix.sigma`.

3. Une pile de pierres presente une face plane de 320 mm : un faux mur tres
   convaincant, mesure a -59 deg de cap quand rien ne l'ecarte. D'ou le
   garde-fou de coherence de `Limits.min_on_border`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class Table:
    """Le rectangle de jeu, en metres. Valeurs officielles Eurobot par defaut."""

    width: float = 3.0
    height: float = 2.0

    def walls(self) -> tuple[np.ndarray, np.ndarray]:
        """Les quatre bordures en distance signee vers l'INTERIEUR : e = n.p + d.

        Le signe de `d` est le piege de ce module. Pour le mur lointain x = L la
        normale interieure vaut (-1, 0), et `d` doit rester POSITIF (+L) pour que
        e(x = L) = 0. Avec -L, e(x = L) = -2L : ce mur n'est jamais associe, le
        recalage ne travaille plus que sur un coin, et le resultat imite a s'y
        meprendre une singularite d'observabilite -- axes « aveugles », biais de
        cap de plusieurs degres, poses sans solution. Deux jours perdus dessus.
        """
        n = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
        d = np.array([0.0, self.width, 0.0, self.height])
        return n, d


@dataclass
class Limits:
    """Reglages du recalage. Les valeurs par defaut sont celles mesurees en v3."""

    gate_start: float = 0.350     # porte d'association initiale, m
    gate_end: float = 0.040       # porte plancher, m
    decay: float = 0.72           # resserrement par iteration, avant la porte adaptative
    iterations: int = 25
    min_points: int = 20          # en dessous, le systeme n'a plus de sens
    min_on_border: int = 60       # coherence : le nuage doit retomber sur le rectangle
    band: float = 0.030           # epaisseur de la bande « sur une bordure », m
    huber: float = 0.008          # echelle plancher des poids de Huber, m
    heading_veto: float = math.pi / 2   # ecart de cap tolere face a `heading`


@dataclass
class Fix:
    """Resultat d'un recalage. `reason` a None veut dire : utilisable."""

    x: float
    y: float
    yaw: float
    covariance: np.ndarray
    inliers: int
    reason: str | None = None

    @property
    def ok(self) -> bool:
        return self.reason is None

    def sigma(self) -> tuple[float, float, float]:
        """Ecart-type par axe, en metres et radians, dans le repere de la table.

        Un axe non contraint ressort a plusieurs milliers de metres : ce n'est
        pas une anomalie, c'est la reponse juste. Le consommateur doit la lire.
        """
        return tuple(float(math.sqrt(max(self.covariance[i, i], 0.0))) for i in range(3))


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def mirror(pose, table: Table = Table()):
    """La pose jumelle, celle que le scan ne sait pas distinguer de `pose`."""
    x, y, yaw = pose
    return table.width - x, table.height - y, wrap(yaw + math.pi)


def match(points, start, table: Table = Table(), limits: Limits = Limits(),
          gate: float | None = None, heading: float | None = None) -> Fix:
    """Recale `points` (dans le repere du robot) sur les bordures de `table`.

    `start` est la pose de depart (x, y, cap) : la prediction de l'odometrie, ou
    une pose connue. `gate` remplace la porte initiale -- elle doit couvrir
    l'ecart que la PREDICTION peut avoir, pas celui du capteur : a 60 mm, une
    odometrie ayant derive de 100 mm depuis la derniere correction n'associe
    plus rien de bon et le recalage rate.

    `heading` est le cap auquel on se fie par CONTINUITE depuis une pose connue
    -- sur Alidade, celui du gyromètre, qui derive de 0.3 deg par minute. Il ne
    sert pas a affiner la solution : il sert uniquement a refuser la pose
    jumelle, a 180 deg. La frontiere est donc large (90 deg par defaut) et la
    marge enorme ; un cap approximatif suffit, un cap absolu n'est pas requis.

    Trois precautions dans la boucle, et aucune n'est facultative :

    - PORTE DECROISSANTE. Fixee a 60 mm, elle rejette d'emblee tout point
      lointain des qu'il y a 3 deg d'erreur de cap au depart : le recalage ne
      mesure plus alors que la porte.
    - PORTE ADAPTATIVE ensuite. Une pile de pierres a 195 mm d'un mur se fait
      associer a ce mur tant que la porte est large, et tire la solution. Passe
      les premieres iterations, on la cale sur la dispersion REELLE des residus
      (4 x MAD) : ce qui depasse est une pierre, pas une bordure.
    - POIDS DE HUBER. Sans lui, un amas de points aberrants pese autant qu'un
      mur entier, et le recalage part a 40 deg -- constate.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    vide = np.zeros((3, 3))
    if len(pts) < limits.min_points:
        return Fix(*start, vide, 0, 'points')
    normales, decalages = table.walls()
    perp = np.column_stack([-pts[:, 1], pts[:, 0]])     # d(R p)/dcap, avant rotation
    x, y, th = start
    porte = limits.gate_start if gate is None else gate
    inliers, Aw, bw, sol = 0, None, None, None
    for k in range(limits.iterations):
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s], [s, c]])
        monde = pts @ R.T + np.array([x, y])
        E = monde @ normales.T + decalages              # (N, 4), distances signees
        idx = np.argmin(np.abs(E), axis=1)
        e = E[np.arange(len(E)), idx]
        garde = np.abs(e) <= porte
        if int(garde.sum()) < limits.min_points:
            return Fix(x, y, th, vide, int(garde.sum()), 'points')
        n = normales[idx[garde]]
        A = np.column_stack([n, np.einsum('ij,ij->i', n, perp[garde] @ R.T)])
        b = -e[garde]
        residus = np.abs(b)
        echelle = max(limits.huber, 1.4826 * float(np.median(residus)))
        w = np.clip(echelle / np.maximum(residus, 1e-9), 0.0, 1.0)
        Aw, bw = A * w[:, None], b * w
        sol, *_ = np.linalg.lstsq(Aw, bw, rcond=None)
        x, y, th = x + sol[0], y + sol[1], wrap(th + sol[2])
        inliers = int((w > 0.5).sum())
        porte = (max(limits.gate_end, min(porte, 4 * echelle)) if k >= 4
                 else max(limits.gate_end, porte * limits.decay))
        if k > 5 and abs(sol[0]) < 1e-6 and abs(sol[1]) < 1e-6 and abs(sol[2]) < 1e-7:
            break

    cov = _covariance(Aw, bw, sol)

    # Coherence : apres recalage, le nuage doit retomber SUR le rectangle. Quand
    # une face de pierre a servi de mur, la solution part se poser ailleurs et
    # plus rien ne touche les bordures -- signature nette d'une divergence.
    c, s = math.cos(th), math.sin(th)
    monde = pts @ np.array([[c, s], [-s, c]]) + np.array([x, y])
    dx = np.minimum(np.abs(monde[:, 0]), np.abs(table.width - monde[:, 0]))
    dy = np.minimum(np.abs(monde[:, 1]), np.abs(table.height - monde[:, 1]))
    if int(np.sum(np.minimum(dx, dy) <= limits.band)) < limits.min_on_border:
        return Fix(x, y, th, cov, inliers, 'divergence')

    # Veto de cap : la seule defense contre la pose jumelle. Elle ne coute rien
    # et elle rend la relocalisation globale sure -- sans elle, il faut
    # l'interdire, et un robot perdu le reste.
    if heading is not None and abs(wrap(th - heading)) > limits.heading_veto:
        return Fix(x, y, th, cov, inliers, 'cap')

    return Fix(x, y, th, cov, inliers, None)


def _covariance(Aw, bw, sol) -> np.ndarray:
    """Covariance 3x3, tiree de l'information que le scan porte REELLEMENT."""
    if Aw is None:
        return np.eye(3) * 1e6
    ddl = max(len(bw) - 3, 1)
    r = bw - Aw @ sol
    var = float(r @ r) / ddl
    try:
        return np.linalg.inv(Aw.T @ Aw + np.eye(3) * 1e-12) * var
    except np.linalg.LinAlgError:
        return np.eye(3) * 1e6
