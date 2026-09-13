"""Pose du cube cible a partir des marqueurs de ses faces, sans ROS.

Chaque face vue donne a elle seule la pose complete du cube : on connait,
par config/cube.yaml, ou est son marqueur dans le repere du cube, et le
detecteur donne ou il est dans celui de la camera. Plusieurs faces vues
donnent donc plusieurs estimations de la meme pose, qu'on moyenne ; leur
dispersion dit si elles s'accordent. Des faces d'accord en position mais pas
en orientation trahissent une face mal decrite dans cube.yaml (motif tourne
d'un quart de tour sur sa face) : c'est ainsi qu'on l'a vu la premiere fois.

Repere d'un marqueur, celui d'OpenCV et d'aruco_opencv : z sort du motif, y
pointe vers le haut du dessin, x = y ^ z. Repere du cube : x vers la face
« devant », y vers la « gauche », z vers le « haut », origine au centre.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass
class Face:
    id: int
    nom: str
    pose: np.ndarray        # 4x4, repere du marqueur dans celui du cube


@dataclass
class CubePose:
    position: np.ndarray    # centre du cube, repere de la camera
    rotation: np.ndarray    # 3x3, axes du cube dans le repere de la camera
    faces: list[int]        # identifiants qui ont contribue
    spread: float           # plus grand ecart d'une face au centre moyen, en m
    angle: float            # plus grand ecart d'une face a l'orientation moyenne, en degres
    method: str = 'orientation'   # 'geometrie' : cap tire des POSITIONS des marqueurs


def load(path: str | Path) -> tuple[float, dict[int, Face]]:
    """(arete, faces par identifiant) depuis cube.yaml."""
    spec = yaml.safe_load(Path(path).read_text())
    edge = float(spec['edge'])
    faces = {}
    for f in spec['faces']:
        z = np.array(f['normal'], float)
        y = np.array(f['up'], float)
        pose = np.eye(4)
        pose[:3, :3] = np.column_stack([np.cross(y, z), y, z])
        pose[:3, 3] = z * edge / 2
        faces[int(f['id'])] = Face(int(f['id']), f['nom'], pose)
    return edge, faces


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


def estimate(markers: dict[int, np.ndarray], faces: dict[int, Face]) -> CubePose | None:
    """Pose du cube depuis les poses 4x4 des marqueurs vus (repere camera).

    Les identifiants etrangers au cube sont ignores : le detecteur voit
    parfois des faux positifs (un 17 a deja ete releve)."""
    poses = [(i, m @ np.linalg.inv(faces[i].pose)) for i, m in sorted(markers.items()) if i in faces]
    if not poses:
        return None
    centres = np.array([p[:3, 3] for _, p in poses])
    centre = centres.mean(axis=0)
    # Moyenne des quaternions, alignes sur le premier : valable tant que les
    # faces s'accordent a quelques degres pres, ce qui est le cas utile.
    quats = [quaternion(p[:3, :3]) for _, p in poses]
    quats = [q if q @ quats[0] >= 0 else -q for q in quats]
    mean = np.sum(quats, axis=0)
    rot = matrix([0, 0, 0], mean / np.linalg.norm(mean))[:3, :3]
    spread = float(np.max(np.linalg.norm(centres - centre, axis=1)))
    angle = max(np.degrees(np.arccos(np.clip((np.trace(rot.T @ p[:3, :3]) - 1) / 2, -1, 1)))
                for _, p in poses)
    return CubePose(centre, rot, [i for i, _ in poses], spread, float(angle))


# --------------------------------------------------------------- cube au sol
TOP_Z = 0.0375      # m : centre de marqueur plus haut = face du dessus (25 ou 50 mm)
PAIR_TOL = 0.008    # m : ecart toleré sur les 35.4 mm entre centres de faces voisines


def _frame(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Base orthonormee directe (a, b, a ^ b) en colonnes ; a et b unitaires, orthogonaux."""
    return np.column_stack([a, b, np.cross(a, b)])


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


def estimate_on_floor(markers: dict[int, np.ndarray], faces: dict[int, Face], edge: float,
                      camera: np.ndarray, single_face_range: float | None = None) -> CubePose | None:
    """Pose d'un cube POSE AU SOL, repere du robot (z vers le haut).

    Un marqueur de 20 px, vu de loin, a deux orientations presque aussi
    probables, et le detecteur choisit la mauvaise la plupart du temps :
    mesure a 70 cm, cube vu en coin, 80 % des images. Sa POSITION, elle, reste
    juste au millimetre. On tire donc la pose des positions des qu'on le peut,
    sans rien supposer de la face qui est en dessus :

    - deux faces verticales voisines : le segment entre leurs centres fait 45
      degres avec chacune ; son sens, et le fait que les deux faces regardent
      forcement la camera, donnent leurs deux normales, donc le cap ;
    - une face verticale et le dessus : le centre du dessus donne le centre du
      cube, et la direction dessus -> face verticale donne la normale de celle-ci ;
    - sinon (une seule face) : son orientation, mais seulement a moins de
      `single_face_range` de la camera. De pres le marqueur est grand et son
      orientation sure (faces d'accord a 1-3 deg a 30 cm) ; de loin elle
      bascule (60 deg a 70 cm, et a un metre ces bascules faisaient changer
      de face visee un cube vu en coin). Au-dela, pas de pose plutot qu'une
      pose fausse.

    Quelle face est dessus ne se lit pas sur l'orientation, douteuse : c'est la
    hauteur mesuree du centre de son marqueur, 50 mm contre 25.
    """
    seen = {i: m for i, m in markers.items() if i in faces}
    if not seen:
        return None
    oriented = estimate(seen, faces)          # repli, et mesure du desaccord des orientations
    up = np.array([0.0, 0.0, 1.0])
    pos = {i: m[:3, 3] for i, m in seen.items()}
    tops = [i for i in seen if pos[i][2] >= TOP_Z]
    sides = [i for i in seen if pos[i][2] < TOP_Z]

    def result(centre, normals_cube, normals_base):
        rot = _frame(*normals_base) @ _frame(*normals_cube).T
        return CubePose(centre, rot, sorted(seen), oriented.spread, oriented.angle, 'geometrie')

    horizontal = lambda v: np.array([v[0], v[1], 0.0])

    # Deux faces verticales voisines.
    for i in sides:
        for j in sides:
            ni, nj = faces[i].pose[:3, 2], faces[j].pose[:3, 2]
            if i >= j or abs(ni @ nj) > 0.1:
                continue                      # meme face ou faces opposees
            d = horizontal(pos[i] - pos[j])
            if abs(np.linalg.norm(d) - edge / np.sqrt(2)) > PAIR_TOL:
                continue                      # appariement incoherent : on n'y croit pas
            u = _unit(d)
            for w in (np.cross(up, u), -np.cross(up, u)):
                Ni, Nj = (u + w) / np.sqrt(2), (w - u) / np.sqrt(2)
                facing = Ni @ (camera - pos[i]) > 0 and Nj @ (camera - pos[j]) > 0
                if facing:
                    centre = (pos[i] + pos[j]) / 2 - (Ni + Nj) * edge / 4
                    centre[2] = edge / 2
                    return result(centre, (ni, nj), (Ni, Nj))

    # Une face verticale et le dessus.
    if tops and sides:
        t, i = tops[0], sides[0]
        centre = pos[t] - up * edge / 2
        Ni = _unit(horizontal(pos[i] - centre))
        nt, ni = faces[t].pose[:3, 2], faces[i].pose[:3, 2]
        if abs(nt @ ni) < 0.1:
            return result(centre, (ni, nt), (Ni, up))

    if single_face_range is not None and \
            min(np.linalg.norm(p - camera) for p in pos.values()) > single_face_range:
        return None
    return oriented


# Faces verticales du cube, par l'axe de sa normale sortante : (colonne de la
# rotation, signe) -> identifiant, pour le dire a l'operateur.
SIDES = {(0, 1): 2, (0, -1): 4, (1, 1): 5, (1, -1): 3}


@dataclass
class DockTarget:
    x: float                # ou base_footprint doit s'arreter, repere du robot
    y: float
    yaw: float              # cap d'arrivee : face au cube
    side: tuple[int, int]   # face abordee, cle de SIDES
    face_id: int


def dock_target(position: np.ndarray, rotation: np.ndarray, edge: float, standoff: float,
                previous: tuple[int, int] | None = None, hysteresis_deg: float = 15.0) -> DockTarget | None:
    """Pose d'arret devant la face verticale la plus tournee vers le robot.

    `position` et `rotation` : le cube dans le repere du robot (base_footprint).
    `standoff` : distance de la face du cube a base_footprint a l'arret, soit
    l'ecart voulu plus la distance de base_footprint a la face avant du robot.
    La face precedente est gardee tant qu'une autre ne fait pas mieux de plus de
    `hysteresis_deg` : cube vu a 45 degres, sans cela, la cible basculerait d'un
    quart de tour d'une image a l'autre en pleine approche. None si le cube
    n'est pas pose sur une face (aucune normale laterale horizontale).
    """
    towards_robot = -np.array([position[0], position[1]])
    towards_robot /= max(np.linalg.norm(towards_robot), 1e-9)
    scores = {}
    for side in SIDES:
        n = rotation[:, side[0]] * side[1]
        if abs(n[2]) > 0.5:
            continue
        horizontal = n[:2] / np.linalg.norm(n[:2])
        scores[side] = (float(horizontal @ towards_robot), horizontal)
    if not scores:
        return None
    best = max(scores, key=lambda s: scores[s][0])
    if previous in scores:
        def angle(side):
            return np.degrees(np.arccos(np.clip(scores[side][0], -1.0, 1.0)))
        if angle(previous) - angle(best) < hysteresis_deg:
            best = previous
    normal = scores[best][1]
    target = position[:2] + normal * (edge / 2 + standoff)
    yaw = float(np.arctan2(-normal[1], -normal[0]))
    return DockTarget(float(target[0]), float(target[1]), yaw, best, SIDES[best])
