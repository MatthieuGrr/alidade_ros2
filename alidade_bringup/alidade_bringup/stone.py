"""Pierres vues par leurs marqueurs : choix de la cible du docking, sans ROS.

Les quatre grandes faces d'une pierre portent le MEME identifiant, et plusieurs
pierres peuvent etre dans le champ : le detecteur rend N marqueurs
indiscernables, dont il faut extraire celui qu'on va aborder. Tout le tri est
geometrique, et il se fait dans le repere du robot, ou le plateau est connu :

  1. REDRESSEMENT. La face d'une pierre est verticale et son motif d'aplomb :
     des six degres de liberte rendus par le detecteur, quatre sont connus
     d'avance. On ne garde que la position du centre -- mesuree au millimetre
     entre 20 et 50 cm -- et le CAP de la normale. L'ambiguite planaire du
     marqueur carre, qui faisait basculer l'orientation du cube de +-8 degres,
     ne peut plus deplacer la cible que dans le plan.
  2. FILTRES. Un marqueur dont la normale n'est pas horizontale, dont le centre
     n'est pas a la hauteur attendue, trop loin, trop pres, ou vu trop de biais,
     n'est pas la face laterale d'une pierre posee devant nous. Chaque rejet
     garde son motif : c'est ce qui s'affiche en rouge dans rviz, et sans lui
     un reglage de seuil se debogue a l'aveugle.
  3. PILES. Deux marqueurs dont les centres se superposent en projection sur le
     plateau appartiennent a la meme pile : seul le plus bas est garde. La
     fenetre de hauteur rejette deja une pierre debout (160 mm) comme la
     deuxieme d'une pile couchee (165 mm), et c'est heureux car ces deux cas
     sont a 5 mm l'un de l'autre ; le regroupement, lui, tient meme si la
     fenetre est ouverte en grand.
  4. VERROUILLAGE. Une fois la cible choisie, elle est gardee tant qu'elle
     reapparait au meme endroit : sans cela, deux pierres a egalite se
     disputeraient la cible d'une image a l'autre en pleine approche.

Repere d'un marqueur, celui d'OpenCV et d'aruco_opencv : z sort du motif, y
pointe vers le haut du dessin, x = y ^ z. Repere du robot : x devant, y a
gauche, z vers le haut, origine au sol.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


# --------------------------------------------------------------- la pierre
@dataclass
class Spec:
    """Description de l'element de jeu, depuis config/stone.yaml."""
    name: str
    size: tuple[float, float, float]    # longueur, largeur, hauteur
    mass: float
    marker_id: int
    marker_size: float                  # cote du carre noir
    marker_quiet: float                 # marge blanche autour
    marker_dict: str

    @property
    def lying(self) -> float:
        """Hauteur du centre du marqueur d'une pierre couchee, face laterale."""
        return self.size[2] / 2

    def stacked(self, level: int) -> float:
        """... de la `level`-ieme d'une pile de pierres couchees (1 = celle du bas)."""
        return self.lying + (level - 1) * self.size[2]

    @property
    def standing(self) -> float:
        """... d'une pierre DEBOUT. A 5 mm de stacked(2) : deux cas qu'aucune
        fenetre de hauteur ne separe, d'ou le regroupement en piles."""
        return self.size[0] / 2

    @property
    def marker_pad(self) -> float:
        """Cote du pave blanc imprime, bordure comprise."""
        return self.marker_size + 2 * self.marker_quiet


def load(path: str | Path) -> Spec:
    spec = yaml.safe_load(Path(path).read_text())
    m = spec['marker']
    return Spec(spec['name'], tuple(float(v) for v in spec['size']), float(spec['mass']),
                int(m['id']), float(m['size']), float(m['quiet']), str(m['dict']))


# ------------------------------------------------------------- outils de pose
def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def matrix(position, quaternion) -> np.ndarray:
    """Pose 4x4 depuis une position et un quaternion (x, y, z, w)."""
    x, y, z, w = quaternion
    pose = np.eye(4)
    pose[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    pose[:3, 3] = position
    return pose


def quaternion(rot: np.ndarray) -> np.ndarray:
    """Quaternion (x, y, z, w), w >= 0, d'une matrice de rotation."""
    w = np.sqrt(max(0.0, 1 + rot[0, 0] + rot[1, 1] + rot[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + rot[0, 0] - rot[1, 1] - rot[2, 2])) / 2, rot[2, 1] - rot[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - rot[0, 0] + rot[1, 1] - rot[2, 2])) / 2, rot[0, 2] - rot[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - rot[0, 0] - rot[1, 1] + rot[2, 2])) / 2, rot[1, 0] - rot[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def face_pose(centre_xy, yaw: float, height: float) -> np.ndarray:
    """Pose 4x4 du marqueur d'une face laterale verticale, dans le repere du robot.

    `yaw` est le cap de la normale SORTANTE. Sert aux essais et aux bancs : c'est
    l'inverse exact de `upright`.
    """
    z = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    y = np.array([0.0, 0.0, 1.0])
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack([np.cross(y, z), y, z])
    pose[:3, 3] = [centre_xy[0], centre_xy[1], height]
    return pose


# ------------------------------------------------------------- les marqueurs
@dataclass
class Tag:
    """Un marqueur detecte, redresse, avec le motif de son rejet s'il y a lieu."""
    centre: np.ndarray      # centre du motif, repere du robot
    yaw: float              # cap de la normale sortante, redressee
    tilt: float             # ecart de la normale a l'horizontale, deg, AVANT redressement
    range: float            # camera -> centre, m
    bearing: float          # azimut du centre vu de la camera, rad
    incidence: float        # angle entre la normale et la ligne de visee, deg
    reason: str | None = None       # None : retenu comme candidat

    @property
    def height(self) -> float:
        return float(self.centre[2])

    @property
    def ok(self) -> bool:
        return self.reason is None


@dataclass
class Limits:
    """Seuils du tri. Tous exposes en parametres ROS, aucun en dur dans le noeud."""
    tilt_max: float = 25.0          # deg, ecart tolere de la normale a l'horizontale
    height_min: float = 0.030       # m, centre du marqueur au-dessus du plateau...
    height_max: float = 0.090       # ... fenetre autour des 55 mm d'une pierre couchee
    range_min: float = 0.10         # m, camera -> marqueur
    range_max: float = 0.90         # au-dela, le cap d'un tag de 80 mm devient faux
    incidence_max: float = 70.0     # deg, face vue trop de biais : pose peu sure
    stack_radius: float = 0.080     # m, centres plus proches que cela : meme pile
    level_tol: float = 0.030        # m, deux marqueurs consideres au meme niveau
    lock_radius: float = 0.100      # m, la cible verrouillee reste la meme tant
                                    # qu'elle reapparait dans ce rayon
    tie: str = 'proche'             # 'proche' ou 'centre' : depart entre pierres a egalite


def upright(pose: np.ndarray, camera: np.ndarray) -> Tag:
    """Marqueur redresse : on ne garde que le centre et le cap de la normale.

    La normale rendue par le detecteur n'est pas exactement horizontale ; son
    ecart (`tilt`) sert a rejeter ce qui n'est pas une face de pierre, mais le
    cap, lui, est pris sur sa projection dans le plan du plateau.
    """
    n = pose[:3, 2]
    tilt = math.degrees(math.asin(min(1.0, abs(float(n[2])))))
    flat = math.hypot(float(n[0]), float(n[1]))
    yaw = math.atan2(float(n[1]), float(n[0])) if flat > 1e-9 else 0.0
    centre = np.array(pose[:3, 3], float)
    to_camera = camera - centre
    distance = float(np.linalg.norm(to_camera))
    bearing = math.atan2(-float(to_camera[1]), -float(to_camera[0]))
    # Incidence : 0 quand on regarde la face de face, 90 quand on la rase.
    cos_i = float(np.dot([math.cos(yaw), math.sin(yaw)], to_camera[:2])) / max(
        float(np.linalg.norm(to_camera[:2])), 1e-9)
    incidence = math.degrees(math.acos(max(-1.0, min(1.0, cos_i))))
    return Tag(centre, yaw, tilt, distance, bearing, incidence)


def assess(detections, spec: Spec, camera: np.ndarray, limits: Limits) -> list[Tag]:
    """Toutes les detections, chacune retenue ou portant le motif de son rejet.

    `detections` : couples (identifiant, pose 4x4 du marqueur dans le repere du
    robot). Les identifiants etrangers sont gardes dans la liste, marques
    'identifiant' : les voir en rouge vaut mieux que de les faire disparaitre,
    un faux positif du detecteur ne se diagnostique pas autrement.
    """
    tags = []
    for marker_id, pose in detections:
        tag = upright(pose, camera)
        if marker_id != spec.marker_id:
            tag.reason = 'identifiant'
        elif tag.tilt > limits.tilt_max:
            tag.reason = 'inclinaison'
        elif not limits.height_min <= tag.height <= limits.height_max:
            tag.reason = 'hauteur'
        elif not limits.range_min <= tag.range <= limits.range_max:
            tag.reason = 'distance'
        elif tag.incidence > limits.incidence_max:
            tag.reason = 'incidence'
        tags.append(tag)
    mark_stacks(tags, limits)
    return tags


def mark_stacks(tags: list[Tag], limits: Limits) -> None:
    """Marque 'empilee' tout marqueur qui en surplombe un autre, retenu et plus bas.

    Compare a l'etat AVANT marquage : dans une pile de trois, la deuxieme est
    rejetee mais continue de rejeter la troisieme, et seule celle du bas reste.
    Deux pierres cote a cote sont a la meme hauteur, et l'inegalite stricte les
    laisse toutes deux candidates.
    """
    standing = [t for t in tags if t.ok]
    for tag in standing:
        for other in standing:
            if other is tag or other.height >= tag.height - 1e-6:
                continue
            if math.dist(tag.centre[:2], other.centre[:2]) < limits.stack_radius:
                tag.reason = 'empilee'
                break


def select(tags: list[Tag], limits: Limits, locked: np.ndarray | None = None) -> Tag | None:
    """Cible : le marqueur retenu le plus BAS, a egalite le plus proche ou le
    plus centre. Une cible deja verrouillee est gardee tant qu'elle reapparait
    a moins de `lock_radius` de sa derniere position."""
    candidates = [t for t in tags if t.ok]
    if not candidates:
        return None
    if locked is not None:
        near = [t for t in candidates if math.dist(t.centre[:2], locked[:2]) < limits.lock_radius]
        if near:
            return min(near, key=lambda t: math.dist(t.centre[:2], locked[:2]))
    lowest = min(t.height for t in candidates)
    level = [t for t in candidates if t.height <= lowest + limits.level_tol]
    key = (lambda t: t.range) if limits.tie == 'proche' else (lambda t: abs(t.bearing))
    return min(level, key=key)


# ------------------------------------------------------------- pose d'arret
@dataclass
class DockTarget:
    x: float                # ou base_footprint doit s'arreter, repere du robot
    y: float
    yaw: float              # cap d'arrivee : face au marqueur
    centre: np.ndarray      # centre du marqueur vise, pour le bras de levier
    standoff: float


def dock_target(tag: Tag, standoff: float) -> DockTarget:
    """Pose d'arret, sur la normale du marqueur, a `standoff` de la face.

    `standoff` : de la FACE de la pierre a base_footprint a l'arret, soit l'ecart
    voulu plus la distance de base_footprint a la face avant du robot. Le centre
    du marqueur etant sur la face, rien a ajouter pour l'epaisseur : c'est toute
    la difference avec le cube, dont la pose se rapportait au centre.
    """
    normal = np.array([math.cos(tag.yaw), math.sin(tag.yaw)])
    stop = tag.centre[:2] + normal * standoff
    return DockTarget(float(stop[0]), float(stop[1]), wrap(tag.yaw + math.pi),
                      tag.centre, standoff)
