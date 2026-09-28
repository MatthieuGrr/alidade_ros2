#!/usr/bin/env python3
"""Outils de reglage de la camera : mise au point, puis calibration a la mire.

    camera_bench.py focus [valeurs...]      # SUR LE ROBOT, mire immobile devant la camera
    camera_bench.py relay                   # SUR LE POSTE, pour cameracalibrator

'focus' fait varier focus_absolute (0 = l'infini) et mesure la nettete de la
mire (variance du laplacien sur son emprise) a chaque valeur, puis remet la
valeur de depart. A lancer a deux distances qui encadrent l'usage ; la valeur
retenue va dans camera.focus de la revision, et impose une calibration : la
focale de la C920 variait de 8 % sur la course de mise au point. Le D435i qui
l'a remplacee n'a pas de mise au point reglable et sort ses intrinseques
d'usine : ce banc sert desormais a mesurer la CADENCE et la LATENCE, pas a
preparer une calibration.

'relay' decode le JPEG du robot en image brute locale sur /calib/image. Le
calibrateur a besoin d'un ecran, donc du poste, et l'image brute (900 ko) ne
passe pas proprement en WiFi ; quant au republish d'image_transport, sous
Jazzy il ne s'abonne jamais a son entree. Pendant la calibration, monter la
qualite JPEG a 95 limite les artefacts sur les coins :
    ros2 param set /camera/camera .color.image_raw.compressed.jpeg_quality 95
"""
import argparse
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

JPEG = '/camera/image_raw/compressed'
PATTERN = (8, 6)      # coins interieurs de la mire de make_checkerboard.py
RELIABLE = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)


def v4l2(device, *args):
    return subprocess.run(['v4l2-ctl', '-d', device, *args],
                          capture_output=True, text=True, check=True).stdout


def cmd_focus(args):
    rclpy.init()
    node = Node('camera_bench_focus')
    last = {}
    node.create_subscription(CompressedImage, JPEG, lambda m: last.update(m=m), RELIABLE)

    def grab(n):
        images, deadline = [], time.time() + 5
        while len(images) < n and time.time() < deadline:
            last.pop('m', None)
            while 'm' not in last and time.time() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if 'm' in last:
                data = np.frombuffer(bytes(last['m'].data), np.uint8)
                images.append(cv2.imdecode(data, cv2.IMREAD_GRAYSCALE))
        return images

    initial = int(v4l2(args.device, '--get-ctrl=focus_absolute').split(':')[1])
    results = []
    try:
        for value in args.values:
            v4l2(args.device, '-c', f'focus_absolute={value}')
            time.sleep(0.8)   # le temps que l'objectif se deplace
            scores, width = [], 0
            for img in grab(3):
                ok, corners = cv2.findChessboardCorners(
                    img, PATTERN, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK)
                if ok:
                    x, y, width, h = cv2.boundingRect(corners.astype(np.float32))
                    scores.append(cv2.Laplacian(img[y:y + h, x:x + width], cv2.CV_64F).var())
            score = float(np.median(scores)) if scores else float('nan')
            results.append((value, score))
            print(f'focus {value:3d} : nettete {score:8.1f}  (mire vue {len(scores)}/3, '
                  f'{width} px de large)', flush=True)
    finally:
        v4l2(args.device, '-c', f'focus_absolute={initial}')
        rclpy.shutdown()
    seen = [r for r in results if r[1] == r[1]]
    if not seen:
        print('mire jamais vue : entiere dans l\'image ? assez eclairee ?')
        return
    best = max(seen, key=lambda r: r[1])
    print(f'meilleur focus {best[0]} ; a plus de 80 % : '
          f'{[v for v, s in seen if s >= 0.8 * best[1]]} (focus remis a {initial})')


def cmd_relay(args):
    rclpy.init()
    node = Node('camera_bench_relay')
    pub = node.create_publisher(Image, args.out, 5)

    def on_jpeg(msg):
        bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if bgr is not None:
            pub.publish(Image(header=msg.header, height=bgr.shape[0], width=bgr.shape[1],
                              encoding='bgr8', is_bigendian=0, step=bgr.shape[1] * 3,
                              data=bgr.tobytes()))

    node.create_subscription(CompressedImage, JPEG, on_jpeg, RELIABLE)
    print(f'{JPEG} -> {args.out} ; dans un autre terminal :\n'
          f'  ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 '
          f'--no-service-check --ros-args -r image:={args.out}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('focus', help='nettete de la mire selon la mise au point')
    p.add_argument('values', nargs='*', type=int, default=list(range(0, 101, 10)))
    p.add_argument('--device', default='/dev/camera')
    p.set_defaults(func=cmd_focus)
    p = sub.add_parser('relay', help='JPEG du robot -> image brute locale')
    p.add_argument('--out', default='/calib/image')
    p.set_defaults(func=cmd_relay)
    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
