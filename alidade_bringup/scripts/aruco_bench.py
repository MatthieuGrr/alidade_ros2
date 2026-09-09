#!/usr/bin/env python3
"""Banc d'essai de la detection ArUco, avec ou sans robot.

    aruco_bench.py watch [duree]            # ce que le detecteur voit vraiment
    aruco_bench.py synth 1 0.50             # camera de synthese : marqueur 1 a 50 cm
    aruco_bench.py sweep                    # portee et justesse sur images parfaites

'synth' publie /camera/image et /camera/camera_info comme le ferait la camera,
ce qui permet de valider toute la chaine sans materiel : dictionnaire, taille
de marqueur, cablage des topics. Le lancer dans un terminal, 'watch' dans un
autre. 'sweep' se passe de ROS et mesure directement dans OpenCV la portee et
l'erreur de distance ; c'est lui qui a justifie l'affinement sous-pixellique
des coins (voir alidade_bringup/config/aruco.yaml).

ATTENTION : les distances ne valent que ce que valent les intrinseques. Tant
que camera_v1.yaml n'a pas ete remplace par une vraie calibration a la mire,
elles portent un biais d'echelle et ignorent la distorsion.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from collections import defaultdict

import cv2
import numpy as np

# Intrinseques nominales, les memes que camera_v1.yaml : les images de synthese
# sont generees et interpretees avec, si bien qu'un ecart mesure ne vient que
# de la localisation des coins, jamais de la calibration.
W, H, FX, CX, CY = 640, 480, 530.0, 320.0, 240.0
SIDE = 0.040          # cote du carre NOIR, comme marker_size


def dictionary():
    """OpenCV 4.6 (Ubuntu 24.04) et 4.7+ n'ont pas la meme API aruco."""
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


def marker_image(dic, mid, px=200):
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dic, mid, px)
    return cv2.aruco.drawMarker(dic, mid, px)


def render(dic, mid, z, margin=60, background=200):
    """Vue synthetique d'un marqueur de face a `z` metres.

    La marge blanche est genereuse a dessein : trop mince, elle forme son
    propre contour sur un fond contraste et le detecteur s'y accroche au lieu
    du carre noir -- le marqueur devient alors invisible de PRES, ce qui est
    contre-intuitif et coute cher a diagnostiquer.
    """
    tag = marker_image(dic, mid)
    tag = cv2.copyMakeBorder(tag, margin, margin, margin, margin,
                             cv2.BORDER_CONSTANT, value=255)
    o = margin
    src = np.float32([[o, o], [o + 199, o], [o + 199, o + 199], [o, o + 199]])
    h = SIDE / 2.0
    dst = np.float32([[FX * x / z + CX, CY - FX * y / z]
                      for x, y in [(-h, h), (h, h), (h, -h), (-h, -h)]])
    scene = cv2.warpPerspective(tag, cv2.getPerspectiveTransform(src, dst),
                                (W, H), borderValue=background)
    return scene, abs(dst[1][0] - dst[0][0])


def cmd_sweep(args):
    dic = dictionary()
    k = np.array([[FX, 0, CX], [0, FX, CY], [0, 0, 1]], float)
    d = np.zeros(5)

    def measure(z, refine):
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            p = cv2.aruco.DetectorParameters_create()
        else:
            p = cv2.aruco.DetectorParameters()
        p.cornerRefinementMethod = (cv2.aruco.CORNER_REFINE_SUBPIX if refine
                                    else cv2.aruco.CORNER_REFINE_NONE)
        corners, ids, _ = cv2.aruco.detectMarkers(render(dic, 1, z)[0], dic, parameters=p)
        if ids is None:
            return None
        _, t, _ = cv2.aruco.estimatePoseSingleMarkers(corners, SIDE, k, d)
        return float(np.linalg.norm(t[0][0]))

    print(f"  {'vrai':>8} {'cote':>8} {'au pixel':>11} {'erreur':>8}   {'sous-pixel':>11} {'erreur':>8}")
    for z in (0.15, 0.20, 0.30, 0.50, 0.75, 1.00, 1.25, 1.50, 2.00):
        px = render(dic, 1, z)[1]
        a, b = measure(z, False), measure(z, True)
        fa = f"{a:.3f} m" if a else "   --   "
        ea = f"{100 * (a - z) / z:+.1f} %" if a else "   -- "
        fb = f"{b:.3f} m" if b else "   --   "
        eb = f"{100 * (b - z) / z:+.1f} %" if b else "   -- "
        print(f"  {z:6.2f} m {px:6.1f}px {fa:>11} {ea:>8}   {fb:>11} {eb:>8}")

    print("\n  decodage des six faces du cube, a 0.50 m :")
    for mid in range(1, 7):
        corners, ids, _ = cv2.aruco.detectMarkers(render(dic, mid, 0.50)[0], dic)
        got = ids.flatten().tolist() if ids is not None else []
        print(f"    identifiant {mid} -> {got if got else 'NON DECODE'}")


def cmd_synth(args):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image

    dic = dictionary()
    scene, px = render(dic, args.marker_id, args.distance)
    rgb = cv2.cvtColor(scene, cv2.COLOR_GRAY2RGB)
    print(f"  marqueur {args.marker_id} a {args.distance:.3f} m, cote apparent {px:.0f} px, "
          f"publication a 15 Hz pendant {args.duration:.0f} s")

    rclpy.init()
    node = Node('aruco_bench_synth')
    pub_img = node.create_publisher(Image, '/camera/image', 5)
    pub_info = node.create_publisher(CameraInfo, '/camera/camera_info', 5)

    info = CameraInfo(width=W, height=H, distortion_model='plumb_bob')
    info.d = [0.0] * 5
    info.k = [FX, 0.0, CX, 0.0, FX, CY, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [FX, 0.0, CX, 0.0, 0.0, FX, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
    img = Image(width=W, height=H, encoding='rgb8', step=W * 3, is_bigendian=0)
    img.data = rgb.tobytes()

    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < args.duration:
        stamp = node.get_clock().now().to_msg()
        img.header.stamp = info.header.stamp = stamp
        img.header.frame_id = info.header.frame_id = 'camera_upright_optical_frame'
        pub_img.publish(img)
        pub_info.publish(info)
        time.sleep(1 / 15.0)
    rclpy.shutdown()


def cmd_watch(args):
    import rclpy
    from aruco_opencv_msgs.msg import ArucoDetection
    from rclpy.node import Node

    rclpy.init()
    node = Node('aruco_bench_watch')
    seen = defaultdict(list)
    stats = {'n': 0, 'with': 0, 'frame': ''}

    def on_detection(msg):
        stats['n'] += 1
        stats['frame'] = msg.header.frame_id
        if msg.markers:
            stats['with'] += 1
        for m in msg.markers:
            p = m.pose.position
            seen[m.marker_id].append((p.x, p.y, p.z))

    node.create_subscription(ArucoDetection, '/aruco_detections', on_detection, 10)
    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.2)
    elapsed = time.monotonic() - start

    print(f"  {stats['n']} images analysees en {elapsed:.0f} s ({stats['n'] / elapsed:.1f} Hz), "
          f"{stats['with']} avec au moins un marqueur, repere '{stats['frame']}'")
    if not seen:
        print("  aucun marqueur vu")
    for mid in sorted(seen):
        pts = seen[mid]
        # Repere optique : x a droite, y vers le bas, z vers l'avant.
        dist = [math.sqrt(x * x + y * y + z * z) for x, y, z in pts]
        moy = sum(dist) / len(dist)
        ecart = math.sqrt(sum((v - moy) ** 2 for v in dist) / len(dist))
        azimut = [math.degrees(math.atan2(x, z)) for x, y, z in pts]
        print(f"  marqueur {mid:2d} : {len(pts):5d} vues ({100 * len(pts) / max(1, stats['n']):5.1f} %), "
              f"distance {moy:.3f} m +/- {ecart * 1000:.0f} mm, "
              f"azimut {sum(azimut) / len(azimut):+.1f} deg")
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='command', required=True)

    w = sub.add_parser('watch', help='ce que publie le detecteur')
    w.add_argument('duration', nargs='?', type=float, default=15.0)
    w.set_defaults(func=cmd_watch)

    s = sub.add_parser('synth', help='camera de synthese')
    s.add_argument('marker_id', type=int)
    s.add_argument('distance', type=float)
    s.add_argument('duration', nargs='?', type=float, default=60.0)
    s.set_defaults(func=cmd_synth)

    p = sub.add_parser('sweep', help='portee et justesse, sans ROS')
    p.set_defaults(func=cmd_sweep)

    args = ap.parse_args()
    return args.func(args) or 0


if __name__ == '__main__':
    sys.exit(main())
