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
