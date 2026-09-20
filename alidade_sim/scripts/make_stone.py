#!/usr/bin/env python3
"""Fabrique le modele Gazebo d'une pierre depuis config/stone.yaml.

    ros2 run alidade_sim make_stone.py            # ecrit dans le paquet source
    ros2 run alidade_sim make_stone.py --out /tmp/stone

Produit `models/stone/` : texture, maillage UV, materiau, SDF. Tout se deduit
de `stone.yaml` -- aucune cote n'est recopiee ici, et regenerer apres un
changement de cote suffit.

POURQUOI UN MAILLAGE ET PAS UNE BOITE. Un `<box>` SDF prend bien une texture,
mais son UV est celui d'Ogre et rien ne garantit ni l'echelle ni le sens sur
chaque face : un marqueur etire ou en MIROIR ne se decode pas, et la panne est
muette. Le maillage porte des UV explicites, verifiables.

POURQUOI L'ATLAS EST SI GRAND. Le carre noir doit rester net au plus pres,
soit 12 cm de la face avant, ou il couvre ~400 px a l'image : la texture est
donc a 6.4 px/mm (512 px pour 80 mm). Gazebo appliquera quand meme ses mipmaps
et adoucira le motif de loin -- c'est le risque connu de cette approche, et
c'est pourquoi `aruco_bench.py stone` reste l'etalon : meme scene, meme
calibration, rendu maitrise. Un ecart entre les deux vient du RENDU, pas de
l'algorithme.

ORIENTATION DES MARQUEURS. Les quatre grandes faces portent le meme motif.
Sur les deux faces verticales d'une pierre couchee, son haut pointe vers le
haut ; sur le dessus et le dessous, vers l'avant de la pierre. Le reglement ne
l'impose pas, mais un motif d'aplomb est ce que `stone.py` suppose en
redressant les poses.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

from alidade_bringup import params, stone

PX_PER_M = 6400.0       # 6.4 px/mm : 512 px pour le carre noir de 80 mm
CARTON = (108, 142, 178)        # BGR, brun carton
PLAIN_PX = 64                   # cote de la plage unie des petites faces


def texture(spec: stone.Spec) -> np.ndarray:
    """Atlas : la grande face en haut, une plage de carton uni en bas."""
    big_w = int(round(spec.size[0] * PX_PER_M))
    big_h = int(round(spec.size[2] * PX_PER_M))
    img = np.zeros((big_h + PLAIN_PX, big_w, 3), np.uint8)
    img[:, :] = CARTON
    pad = int(round(spec.marker_pad * PX_PER_M))
    tag = int(round(spec.marker_size * PX_PER_M))
    if pad > min(big_w, big_h):
        raise SystemExit(f'le pave de {spec.marker_pad * 1000:.0f} mm ne tient pas sur la face')
    for size, fill in ((pad, None), (tag, 'tag')):
        r, c = (big_h - size) // 2, (big_w - size) // 2
        if fill is None:
            img[r:r + size, c:c + size] = 255
        else:
            dic = (cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f'DICT_{spec.marker_dict}'))
                   if hasattr(cv2.aruco, 'getPredefinedDictionary') else
                   cv2.aruco.Dictionary_get(getattr(cv2.aruco, f'DICT_{spec.marker_dict}')))
            motif = (cv2.aruco.generateImageMarker(dic, spec.marker_id, size)
                     if hasattr(cv2.aruco, 'generateImageMarker') else
                     cv2.aruco.drawMarker(dic, spec.marker_id, size))
            img[r:r + size, c:c + size] = cv2.cvtColor(motif, cv2.COLOR_GRAY2BGR)
    return img


def mesh(spec: stone.Spec, shape) -> str:
    """Maillage UV de la boite. Les UV sont poses face par face, en regardant
    chaque face DE L'EXTERIEUR, pour qu'aucun motif ne sorte en miroir."""
    hx, hy, hz = (v / 2 for v in spec.size)
    # v est compte depuis le BAS de l'image en OBJ : la grande face occupe le
    # haut de l'atlas, donc le haut des v.
    total_h, big_w = shape[0], shape[1]
    v0 = 1.0 - (total_h - PLAIN_PX) / total_h
    big = [(0.0, v0), (1.0, v0), (1.0, 1.0), (0.0, 1.0)]     # bas-gauche -> haut-gauche
    # Au coeur de la plage unie, loin des bords : les mipmaps melangent les
    # texels voisins et un UV colle au bord ramenerait du blanc sur le carton.
    u1, v1 = 0.2 * PLAIN_PX / big_w, 0.2 * v0
    u2, v2 = 0.8 * PLAIN_PX / big_w, 0.8 * v0
    plain = [(u1, v1), (u2, v1), (u2, v2), (u1, v2)]

    def corners(origin, right, up, half_r, half_u):
        """Les 4 sommets d'une face, dans l'ordre bas-gauche -> haut-gauche VUE
        DE L'EXTERIEUR : c'est cet ordre qui fixe le sens du motif."""
        o, r, u = np.array(origin), np.array(right), np.array(up)
        return [o - r * half_r - u * half_u, o + r * half_r - u * half_u,
                o + r * half_r + u * half_u, o - r * half_r + u * half_u]

    faces = []
    # Deux faces verticales d'une pierre couchee : le haut du motif vers le haut.
    # Vue de l'exterieur, la droite de l'image est -x pour +y, +x pour -y.
    faces.append((corners([0, hy, 0], [-1, 0, 0], [0, 0, 1], hx, hz), [0, 1, 0], big))
    faces.append((corners([0, -hy, 0], [1, 0, 0], [0, 0, 1], hx, hz), [0, -1, 0], big))
    # Dessus et dessous : le haut du motif vers l'avant de la pierre (+x).
    faces.append((corners([0, 0, hz], [0, -1, 0], [1, 0, 0], hy, hx), [0, 0, 1], big))
    faces.append((corners([0, 0, -hz], [0, 1, 0], [1, 0, 0], hy, hx), [0, 0, -1], big))
    # Les deux petites faces (110 x 110) ne portent RIEN : carton uni.
    faces.append((corners([hx, 0, 0], [0, 1, 0], [0, 0, 1], hy, hz), [1, 0, 0], plain))
    faces.append((corners([-hx, 0, 0], [0, -1, 0], [0, 0, 1], hy, hz), [-1, 0, 0], plain))

    out = ['# Pierre de la Coupe de France, engendre par alidade_sim/scripts/make_stone.py',
           '# Ne pas editer : regenerer apres tout changement de config/stone.yaml.',
           'mtllib stone.mtl', 'o stone']
    vt_lines, vn_lines, f_lines = [], [], []
    for i, (pts, normal, uv) in enumerate(faces):
        for p in pts:
            out.append(f'v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}')
        for u, v in uv:
            vt_lines.append(f'vt {u:.6f} {v:.6f}')
        vn_lines.append(f'vn {normal[0]} {normal[1]} {normal[2]}')
        a, b, c, d = (4 * i + 1, 4 * i + 2, 4 * i + 3, 4 * i + 4)
        n = i + 1
        f_lines.append(f'f {a}/{a}/{n} {b}/{b}/{n} {c}/{c}/{n}')
        f_lines.append(f'f {a}/{a}/{n} {c}/{c}/{n} {d}/{d}/{n}')
    out += vt_lines + vn_lines + ['usemtl carton', 's off'] + f_lines
    return '\n'.join(out) + '\n'


def sdf(spec: stone.Spec) -> str:
    """Modele SDF : collision en boite, visuel maille, inertie homogene."""
    x, y, z = spec.size
    m = spec.mass
    ixx = m / 12 * (y * y + z * z)
    iyy = m / 12 * (x * x + z * z)
    izz = m / 12 * (x * x + y * y)
    return f'''<?xml version="1.0"?>
<!--
  Engendre par alidade_sim/scripts/make_stone.py depuis
  alidade_bringup/config/stone.yaml. Ne pas editer : regenerer.

  {m * 1000:.0f} g pour {x * 1000:.0f} x {y * 1000:.0f} x {z * 1000:.0f} mm : la pierre se
  renverse et glisse pour un rien, et c'est voulu. L'approche doit s'arreter
  AVANT de la toucher. Frottement de carton sur vinyle (mu 0.4, valeur usuelle
  0.3 a 0.5) ; la collision est une boite, le maillage ne sert qu'a porter les
  marqueurs.

  XML interdit deux tirets consecutifs dans un commentaire : les eviter ici,
  sdformat les tolere mais ElementTree non, et le test du modele echoue.
-->
<sdf version="1.9">
  <model name="stone">
    <link name="link">
      <inertial>
        <mass>{m}</mass>
        <inertia>
          <ixx>{ixx:.8f}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy:.8f}</iyy><iyz>0</iyz>
          <izz>{izz:.8f}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{x} {y} {z}</size></box></geometry>
        <surface>
          <friction><ode><mu>0.4</mu><mu2>0.4</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>model://stone/meshes/stone.obj</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
'''


CONFIG = '''<?xml version="1.0"?>
<model>
  <name>stone</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Pierre de la Coupe de France de robotique. Engendre par
  alidade_sim/scripts/make_stone.py.</description>
</model>
'''


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', help='repertoire du modele (defaut : le paquet source)')
    args = ap.parse_args()

    spec = stone.load(params.bringup_share() / 'config' / 'stone.yaml')
    out = Path(args.out) if args.out else (
        Path(__file__).resolve().parent.parent / 'models' / 'stone')
    (out / 'meshes').mkdir(parents=True, exist_ok=True)
    (out / 'materials' / 'textures').mkdir(parents=True, exist_ok=True)

    img = texture(spec)
    cv2.imwrite(str(out / 'materials' / 'textures' / 'stone.png'), img)
    (out / 'meshes' / 'stone.obj').write_text(mesh(spec, img.shape))
    (out / 'meshes' / 'stone.mtl').write_text(
        'newmtl carton\nKa 1.0 1.0 1.0\nKd 1.0 1.0 1.0\nKs 0.0 0.0 0.0\nd 1.0\nillum 1\n'
        'map_Kd ../materials/textures/stone.png\n')
    (out / 'model.sdf').write_text(sdf(spec))
    (out / 'model.config').write_text(CONFIG)

    print(f'  {out}')
    print(f'  texture      {img.shape[1]} x {img.shape[0]} px, '
          f'{PX_PER_M / 1000:.1f} px/mm, carre noir {int(spec.marker_size * PX_PER_M)} px')
    print(f'  maillage     boite {"x".join(f"{v * 1000:.0f}" for v in spec.size)} mm, '
          f'6 faces UV, marqueur {spec.marker_id} sur les 4 grandes')
    print(f'  modele       {spec.mass * 1000:.0f} g, inertie de boite homogene')
    return 0


if __name__ == '__main__':
    sys.exit(main())
