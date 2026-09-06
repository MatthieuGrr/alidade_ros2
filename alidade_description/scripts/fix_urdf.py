#!/usr/bin/env python3
"""Corrige un URDF exporte depuis Onshape a partir du fichier de revision materielle.

L'exporteur d'Onshape ne sait pas produire :

  - une racine nommee : il fabrique un link synthetique 'root' ;
  - des joints distincts des links homonymes : URDF le tolere, SDF non ;
  - des masses reelles : les materiaux ne sont pas transmis, tout sort a
    une densite de 1 kg/m3. Cette densite etant uniforme, chaque tenseur
    d'inertie est geometriquement juste a un facteur pres, et le corriger
    revient a le multiplier par m_mesuree / m_exportee ;
  - des geometries de collision.

Le script rejoue ces corrections a chaque build a partir de la revision
materielle, verifie le centre de masse et les poses des reperes principaux,
puis ecrit un URDF purement descriptif. Tout ce qui depend de la cible
(ros2_control, capteurs Gazebo) est ajoute ensuite par le xacro.

    fix_urdf.py --revision config/alidade_v1.yaml --urdf-dir urdf \
                --package alidade_description --output generated/alidade_v1.urdf
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

INERTIA_KEYS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")


def rename_link(robot: ET.Element, old: str, new: str) -> int:
    """Renomme un link et toutes les references de joint qui le designent."""
    hits = 0
    for link in robot.findall("link"):
        if link.get("name") == old:
            link.set("name", new)
            hits += 1
    for joint in robot.findall("joint"):
        for tag in ("parent", "child"):
            ref = joint.find(tag)
            if ref is not None and ref.get("link") == old:
                ref.set("link", new)
                hits += 1
    return hits


def rename_joint(robot: ET.Element, old: str, new: str) -> bool:
    for joint in robot.findall("joint"):
        if joint.get("name") == old:
            joint.set("name", new)
            return True
    return False


def rewrite_mesh_uris(robot: ET.Element, package: str) -> int:
    """Le nom de paquet que l'exporteur met dans package:// est celui saisi
    dans Onshape au moment de l'export ; on le ramene au paquet courant pour
    qu'un reexport fonctionne quel que soit ce nom."""
    hits = 0
    pattern = re.compile(r"^package://[^/]+/")
    for mesh in robot.iter("mesh"):
        filename = mesh.get("filename", "")
        replaced = pattern.sub(f"package://{package}/", filename)
        if replaced != filename:
            mesh.set("filename", replaced)
            hits += 1
    return hits


def apply_mass(link: ET.Element, target: float) -> tuple[float, float]:
    """Impose une masse et met le tenseur d'inertie a la meme echelle."""
    inertial = link.find("inertial")
    if inertial is None:
        raise SystemExit(f"{link.get('name')} : aucun bloc <inertial> a corriger")

    mass_el = inertial.find("mass")
    exported = float(mass_el.get("value"))
    if exported <= 0.0:
        raise SystemExit(
            f"{link.get('name')} : masse exportee nulle, mise a l'echelle impossible"
        )

    ratio = target / exported
    mass_el.set("value", f"{target:.6g}")

    inertia = inertial.find("inertia")
    for key in INERTIA_KEYS:
        inertia.set(key, f"{float(inertia.get(key)) * ratio:.6e}")

    return exported, ratio


def rotation(rpy: list[float]) -> list[list[float]]:
    cr, cp, cy = (math.cos(a) for a in rpy)
    sr, sp, sy = (math.sin(a) for a in rpy)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rpy_from_rotation(rot: list[list[float]]) -> list[float]:
    """Inverse de rotation() : rpy URDF (Rz.Ry.Rx) depuis la matrice."""
    sp = max(-1.0, min(1.0, -rot[2][0]))
    pitch = math.asin(sp)
    if abs(sp) > 1.0 - 1e-9:  # gimbal lock : roulis et lacet ne sont plus separables
        return [0.0, pitch, math.atan2(-rot[0][1], rot[1][1])]
    return [
        math.atan2(rot[2][1], rot[2][2]),
        pitch,
        math.atan2(rot[1][0], rot[0][0]),
    ]


def rotate_frame(robot: ET.Element, child: str, extra: list[float]) -> str:
    """Compose une rotation dans le repere d'un link, sur le joint qui le porte.

    L'export decrit ou la CAO pose un capteur, jamais l'orientation du repere
    dans lequel il mesure. Quand les deux different -- le 0 d'un lidar qui ne
    tombe pas du cote suppose -- c'est ici qu'on les reconcilie. La rotation est
    appliquee a droite, donc exprimee dans le repere de l'enfant, et laisse la
    position du joint intacte : les poses verifiees par check_frames ne bougent
    pas."""
    for joint in robot.findall("joint"):
        if joint.find("child").get("link") != child:
            continue
        origin = joint.find("origin")
        base = rotation([float(v) for v in origin.get("rpy").split()])
        turn = rotation(extra)
        composed = [
            [sum(base[i][k] * turn[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]
        rpy = rpy_from_rotation(composed)
        origin.set("rpy", " ".join(f"{v:.17g}" for v in rpy))
        return joint.get("name")
    raise SystemExit(f"aucun joint ne porte le link '{child}'")


class Tree:
    """Poses des links dans le repere racine, calculees depuis les joints."""

    def __init__(self, robot: ET.Element):
        self.parents = {}
        for joint in robot.findall("joint"):
            origin = joint.find("origin")
            self.parents[joint.find("child").get("link")] = (
                joint.find("parent").get("link"),
                [float(v) for v in origin.get("xyz").split()],
                [float(v) for v in origin.get("rpy").split()],
            )

    def pose(self, name: str) -> tuple[list[list[float]], list[float]]:
        rot = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
        pos = [0.0, 0.0, 0.0]
        while name in self.parents:
            name, xyz, rpy = self.parents[name]
            up = rotation(rpy)
            pos = [sum(up[i][k] * pos[k] for k in range(3)) + xyz[i] for i in range(3)]
            rot = [
                [sum(up[i][k] * rot[k][j] for k in range(3)) for j in range(3)]
                for i in range(3)
            ]
        return rot, pos


def centre_of_mass(robot: ET.Element, tree: Tree) -> tuple[float, list[float]]:
    total = 0.0
    weighted = [0.0, 0.0, 0.0]
    for link in robot.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        local = [float(v) for v in inertial.find("origin").get("xyz").split()]
        rot, pos = tree.pose(link.get("name"))
        world = [sum(rot[i][k] * local[k] for k in range(3)) + pos[i] for i in range(3)]
        total += mass
        weighted = [weighted[i] + mass * world[i] for i in range(3)]

    com = [v / total for v in weighted] if total else weighted
    return total, com


def add_collision(link: ET.Element, spec: dict) -> str:
    """Primitive de collision a la place du maillage visuel."""
    for existing in link.findall("collision"):
        link.remove(existing)

    collision = ET.SubElement(link, "collision")
    origin = ET.SubElement(collision, "origin")
    origin.set("xyz", " ".join(f"{v:g}" for v in spec.get("origin", [0, 0, 0])))
    origin.set("rpy", " ".join(f"{v:g}" for v in spec.get("rpy", [0, 0, 0])))

    geometry = ET.SubElement(collision, "geometry")
    shape = spec["shape"]
    if shape == "box":
        ET.SubElement(geometry, "box").set("size", " ".join(f"{v:g}" for v in spec["size"]))
        detail = "x".join(f"{v * 1000:.0f}" for v in spec["size"]) + " mm"
    elif shape == "cylinder":
        node = ET.SubElement(geometry, "cylinder")
        node.set("radius", f"{spec['radius']:g}")
        node.set("length", f"{spec['length']:g}")
        detail = f"r={spec['radius'] * 1000:.1f} l={spec['length'] * 1000:.1f} mm"
    elif shape == "sphere":
        ET.SubElement(geometry, "sphere").set("radius", f"{spec['radius']:g}")
        detail = f"r={spec['radius'] * 1000:.1f} mm"
    else:
        raise SystemExit(f"{link.get('name')} : forme '{shape}' inconnue")
    return f"{shape:<10}{detail}"


def check_frames(tree: Tree, spec: dict) -> None:
    """Compare les poses de l'export aux valeurs attendues de la revision.

    Un reexport Onshape qui a deplace un capteur passerait sinon inapercu
    jusqu'a ce que le SLAM derive."""
    tol = float(spec.get("tolerance", 0.001))
    print(f"\n{'reperes':<24}{'x':>9}{'y':>9}{'z':>9}{'ecart':>9}")
    print("-" * 71)
    worst = 0.0
    bad = []
    for name, expected in spec["expected"].items():
        try:
            _, pos = tree.pose(name)
        except KeyError:
            raise SystemExit(f"repere '{name}' absent de l'export")
        if name not in tree.parents:
            raise SystemExit(f"repere '{name}' absent de l'export")
        err = math.dist(pos, expected)
        worst = max(worst, err)
        flag = "" if err <= tol else "  ECART"
        if err > tol:
            bad.append(name)
        print(f"{name:<24}{pos[0]:>+9.4f}{pos[1]:>+9.4f}{pos[2]:>+9.4f}{err * 1000:>7.2f} mm{flag}")
    print(f"{'ecart maximal':<24}{'':>27}{worst * 1000:>7.2f} mm  (tolerance {tol * 1000:.1f} mm)")
    if bad:
        raise SystemExit(
            f"poses hors tolerance : {', '.join(bad)} -- l'export Onshape a bouge, "
            "ou la revision materielle est a mettre a jour"
        )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--revision", type=Path, required=True, help="config/alidade_<rev>.yaml")
    ap.add_argument("--urdf-dir", type=Path, required=True, help="dossier urdf/ du paquet")
    ap.add_argument("--package", required=True, help="nom du paquet pour les URI package://")
    ap.add_argument("--output", type=Path, required=True, help="URDF corrige a ecrire")
    args = ap.parse_args()

    config = yaml.safe_load(args.revision.read_text())
    source = args.urdf_dir / config["onshape_export"]
    tree_xml = ET.parse(source)
    robot = tree_xml.getroot()
    robot.set("name", "alidade")

    print(f"revision    {config.get('revision')}  ({args.revision})")
    print(f"source      {source}\n")

    rename = config.get("rename_root")
    if rename:
        hits = rename_link(robot, rename["from"], rename["to"])
        if hits:
            print(f"racine      {rename['from']} -> {rename['to']}  ({hits} references)")
        else:
            print(f"racine      '{rename['from']}' introuvable, rien a renommer")

    for old, new in (config.get("rename_joints") or {}).items():
        state = "renomme" if rename_joint(robot, old, new) else "introuvable"
        print(f"joint       {old} -> {new}  ({state})")

    names_l = {l.get("name") for l in robot.findall("link")}
    names_j = {j.get("name") for j in robot.findall("joint")}
    clash = sorted(names_l & names_j)
    if clash:
        raise SystemExit(
            f"noms partages link/joint : {', '.join(clash)} -- URDF l'accepte, "
            "Gazebo refusera le modele ; ajouter une entree rename_joints"
        )
    print("aucun nom partage entre links et joints")

    print(f"maillages   {rewrite_mesh_uris(robot, args.package)} URI ramenees a package://{args.package}/")

    # Avant Tree() : l'arbre sert ensuite au centre de masse et a check_frames.
    lidar = config.get("lidar") or {}
    mount_rpy = lidar.get("mount_rpy")
    if mount_rpy:
        joint = rotate_frame(robot, lidar["frame"], [float(v) for v in mount_rpy])
        degres = ", ".join(f"{math.degrees(float(v)):+.1f}" for v in mount_rpy)
        print(f"repere      {lidar['frame']} tourne de ({degres}) deg sur {joint}")
    print()

    links = {link.get("name"): link for link in robot.findall("link")}
    masses = config.get("masses", {})
    missing = sorted(set(masses) - set(links))
    if missing:
        raise SystemExit(f"links absents de l'URDF : {', '.join(missing)}")

    print(f"{'link':<24}{'exporte':>12}{'mesure':>12}{'facteur':>11}{'densite':>12}")
    print("-" * 71)
    for name in sorted(masses):
        exported, ratio = apply_mass(links[name], float(masses[name]))
        volume_m3 = exported  # densite d'export = 1 kg/m3
        density = float(masses[name]) / volume_m3 / 1000.0
        print(
            f"{name:<24}{exported * 1000:>10.4f} g{masses[name] * 1000:>10.1f} g"
            f"{ratio:>11.1f}{density:>10.2f} g/cm3"
        )

    untouched = sum(
        float(link.find("inertial/mass").get("value"))
        for name, link in links.items()
        if name not in masses and link.find("inertial") is not None
    )
    tree = Tree(robot)
    total, com = centre_of_mass(robot, tree)

    print("-" * 71)
    print(f"{'links non listes':<24}{untouched * 1000:>10.4f} g   (porteurs de repere)")
    print(f"{'TOTAL':<24}{total * 1000:>10.1f} g")
    print(f"\ncentre de masse  x={com[0]:+.4f}  y={com[1]:+.4f}  z={com[2]:+.4f}  (repere racine)")
    if abs(com[1]) > 0.005:
        raise SystemExit(
            "centre de masse decale de plus de 5 mm sur Y : une des trois pesees "
            "ou l'export est incoherent"
        )

    collisions = config.get("collisions", {})
    if collisions:
        missing = sorted(set(collisions) - set(links))
        if missing:
            raise SystemExit(f"links absents de l'URDF : {', '.join(missing)}")
        print(f"\n{'collisions':<24}{'primitive':<10}{'dimensions'}")
        print("-" * 71)
        for name in sorted(collisions):
            print(f"{name:<24}{add_collision(links[name], collisions[name])}")

    frames = config.get("frames")
    if frames:
        check_frames(tree, frames)

    ET.indent(tree_xml, space="    ")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree_xml.write(args.output, encoding="utf-8", xml_declaration=True)
    print(f"\necrit       {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
