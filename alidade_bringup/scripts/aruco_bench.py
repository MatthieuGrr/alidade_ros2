#!/usr/bin/env python3
"""Banc d'essai de la detection ArUco, avec ou sans robot.

    aruco_bench.py watch [duree]              # ce que le detecteur voit vraiment
    aruco_bench.py synth 13 0.50              # camera de synthese : marqueur a 50 cm
    aruco_bench.py sweep                      # portee et justesse sur images parfaites
    aruco_bench.py stone                      # batterie de scenes : la bonne pierre est-elle choisie ?
    aruco_bench.py stone 0.40,0,90            # ... ou une scene donnee : x,y,cap[,niveau]
    aruco_bench.py stone-range                # enveloppe de visibilite selon l'inclinaison
    aruco_bench.py fake-stones 0.6,0,90       # SIMULATION : pierres posees dans odom

'synth' publie /camera/image_raw et /camera/camera_info comme le ferait la camera,
ce qui permet de valider toute la chaine sans materiel : dictionnaire, taille
de marqueur, cablage des topics. Le lancer dans un terminal, 'watch' dans un
autre. 'sweep' se passe de ROS et mesure directement dans OpenCV la portee et
l'erreur de distance ; c'est lui qui a justifie l'affinement sous-pixellique
des coins (voir alidade_bringup/config/aruco.yaml).

'stone' et 'stone-range' rendent des pierres avec les VRAIES intrinseques et la
camera a sa vraie hauteur : ils mesurent ce que le robot verra, et 'stone-range'
montre a quel point l'inclinaison du support decide de tout. Ils servent
d'etalon au modele Gazebo : un ecart entre les deux vient du rendu, pas de
l'algorithme.

ATTENTION : sur le robot, les distances ne valent que ce que valent les
intrinseques de camera_v1.yaml, donc la calibration a la mire et la mise au
point avec laquelle elle a ete faite (voir camera_bench.py).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from collections import defaultdict, deque

import cv2
import numpy as np

from alidade_bringup import params, stone

# Intrinseques de synthese, volontairement independantes de camera_v1.yaml :
# les images sont generees et interpretees avec, si bien qu'un ecart mesure ne
# vient que de la localisation des coins, jamais de la calibration.
W, H, FX, CX, CY = 640, 480, 530.0, 320.0, 240.0
SIDE = 0.080          # cote du carre NOIR, comme marker_size


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

    print("\n  decodage des identifiants voisins de celui des pierres, a 0.50 m :")
    for mid in range(11, 16):
        corners, ids, _ = cv2.aruco.detectMarkers(render(dic, mid, 0.50)[0], dic)
        got = ids.flatten().tolist() if ids is not None else []
        print(f"    identifiant {mid} -> {got if got else 'NON DECODE'}"
              + ('   <- celui des pierres' if mid == 13 else ''))


# --------------------------------------------------------- rendu d'une pierre
# Ici, contrairement a `sweep`, on rend avec les VRAIES intrinseques et la
# camera a sa vraie place : c'est l'enveloppe de visibilite du ROBOT qu'on
# mesure. Le surechantillonnage n'est pas un luxe : sans lui, warpPerspective
# replie la texture et le marqueur devient indecodable de 15 a 80 cm -- une
# panne de RENDU qu'on prendrait pour une limite de portee.
SS = 4                  # surechantillonnage du rendu
PX_PER_M = 5120         # texture : ~410 px pour le carre noir de 80 mm
PATCH = (0.200, 0.110)  # portion de face plaquee, centree sur le marqueur. Au-dela
                        # le carton est uniforme et ne change rien au decodage, et
                        # une face de 320 mm traverse le plan de la camera de pres.
CARTON, FOND = 150, 190


def detector_params(refine=True):
    """Parametres du detecteur, ceux de config/aruco.yaml."""
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        p = cv2.aruco.DetectorParameters_create()
    else:
        p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = (cv2.aruco.CORNER_REFINE_SUBPIX if refine
                                else cv2.aruco.CORNER_REFINE_NONE)
    p.cornerRefinementWinSize, p.cornerRefinementMaxIterations = 5, 30
    p.cornerRefinementMinAccuracy = 0.1
    p.minMarkerDistanceRate = 0.03      # voir config/aruco.yaml : hors de 0.02-0.044,
    return p                            # le marqueur est perdu ou compte double


def bench_camera(revision, tilt_deg):
    """(K, distorsion, (largeur, hauteur), pose 4x4 de la camera dans base_footprint).

    La position vient de la revision materielle, l'inclinaison est donnee : elle
    depend du support, et c'est justement ce qu'on veut faire varier.
    """
    import yaml
    hw = params.load_revision(revision)
    calib = yaml.safe_load(
        (params.description_share() / 'config' / hw['camera']['calibration']).read_text())
    k = np.array(calib['camera_matrix']['data'], float).reshape(3, 3)
    dist = np.array(calib['distortion_coefficients']['data'], float)
    x, _, z = hw['frames']['expected']['camera_optical_frame']
    t = math.radians(tilt_deg)
    # Repere optique : z le long de l'axe (avant, incline vers le sol), x a
    # droite, y vers le bas de l'image.
    axis = np.array([math.cos(t), 0.0, -math.sin(t)])
    right = np.array([0.0, -1.0, 0.0])
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack([right, np.cross(axis, right), axis])
    pose[:3, 3] = [x, 0.0, z]
    return k, dist, (calib['image_width'], calib['image_height']), pose, hw


def face_texture(dic, spec, marker_id):
    """Pave de carton portant le marqueur et sa marge blanche."""
    n_tag = int(spec.marker_size * PX_PER_M)
    n_pad = int(spec.marker_pad * PX_PER_M)
    tex = np.full((int(PATCH[1] * PX_PER_M), int(PATCH[0] * PX_PER_M)), CARTON, np.uint8)
    for size, fill in ((n_pad, 255), (n_tag, None)):
        r, c = (tex.shape[0] - size) // 2, (tex.shape[1] - size) // 2
        tex[r:r + size, c:c + size] = 255 if fill else marker_image(dic, marker_id, size)
    return tex


def render_stones(stones, spec, cam, noise=1.5):
    """Image que la camera verrait. `stones` : (x, y, cap deg, hauteur du
    marqueur, identifiant), le CAP etant celui du GRAND AXE de la pierre -- 90
    degres met donc une grande face en face du robot.

    Les faces sont peintes de la plus lointaine a la plus proche : une pierre de
    320 mm a 40 cm en masque une autre derriere elle, comme en vrai.
    """
    k, dist, (w, h), base_from_opt = cam
    dic = dictionary()
    cam_from_base = np.linalg.inv(base_from_opt)
    ks = k.copy(); ks[:2] *= SS
    scene = np.full((h * SS, w * SS), FOND, np.uint8)
    rvec, tvec = cv2.Rodrigues(np.eye(3))[0], np.zeros(3)
    half_w, half_h = PATCH[0] / 2, PATCH[1] / 2
    quad = np.array([[-half_w, half_h, 0], [half_w, half_h, 0],
                     [half_w, -half_h, 0], [-half_w, -half_h, 0]], float)
    for (x, y, yaw, height, mid) in sorted(stones, key=lambda s: -math.hypot(s[0], s[1])):
        # Les deux grandes faces sont perpendiculaires au grand axe.
        for side in (1, -1):
            normal = math.radians(yaw) + side * math.pi / 2
            centre = np.array([x, y]) + (spec.size[1] / 2) * np.array(
                [math.cos(normal), math.sin(normal)])
            in_cam = cam_from_base @ stone.face_pose(centre, normal, height)
            if in_cam[2, 3] <= 0.02 or in_cam[:3, 2] @ (-in_cam[:3, 3]) <= 0:
                continue        # derriere la camera, ou face tournee a l'oppose
            pts = in_cam[:3, :3] @ quad.T + in_cam[:3, 3:4]
            if np.any(pts[2] < 0.02):
                continue        # la face traverse le plan de la camera
            dst = cv2.projectPoints(pts.T, rvec, tvec, ks, dist)[0].reshape(-1, 2).astype(np.float32)
            tex = face_texture(dic, spec, mid)
            th, tw = tex.shape
            src = np.float32([[0, 0], [tw - 1, 0], [tw - 1, th - 1], [0, th - 1]])
            warp = cv2.warpPerspective(tex, cv2.getPerspectiveTransform(src, dst),
                                       (w * SS, h * SS), borderValue=255)
            mask = np.zeros((h * SS, w * SS), np.uint8)
            cv2.fillConvexPoly(mask, dst.astype(np.int32), 255)
            scene[mask > 0] = warp[mask > 0]
    scene = cv2.resize(scene, (w, h), interpolation=cv2.INTER_AREA)
    if noise:
        scene = np.clip(scene.astype(np.int16) + np.random.normal(0, noise, scene.shape),
                        0, 255).astype(np.uint8)
    return scene


def detect_stones(scene, spec, cam):
    """(identifiant, pose 4x4 dans base_footprint) pour chaque marqueur decode."""
    k, dist, _, base_from_opt = cam
    corners, ids, _ = cv2.aruco.detectMarkers(scene, dictionary(), parameters=detector_params())
    out = []
    if ids is None:
        return out
    half = spec.marker_size / 2
    obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], float)
    for mid, quad in zip(ids.flatten().tolist(), corners):
        ok, rvec, tvec = cv2.solvePnP(obj, quad.reshape(-1, 2), k, dist,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        pose = np.eye(4)
        pose[:3, :3] = cv2.Rodrigues(rvec)[0]
        pose[:3, 3] = tvec.ravel()
        out.append((mid, base_from_opt @ pose))
    return out


# Batterie de reference : chaque cas donne les pierres posees et le centre de
# celle qu'il FAUT viser (None : aucune). Les placements tiennent compte de la
# demi-ouverture horizontale de 26.4 degres -- deux pierres ecartees de 16 cm a
# 35 cm sortent du champ, ce n'est pas un defaut de la selection.
def battery(spec):
    low, l2, l3 = spec.lying, spec.stacked(2), spec.stacked(3)
    return [
        ('pierre seule, de face',       [(0.40, 0.00, 90, low, 13)], (0.40, 0.00)),
        ('pierre seule, de travers 20', [(0.40, 0.00, 70, low, 13)], (0.40, 0.00)),
        ('pierre seule au plus pres',   [(0.30, 0.00, 90, low, 13)], (0.30, 0.00)),
        ('pierre seule a 70 cm',        [(0.70, 0.00, 90, low, 13)], (0.70, 0.00)),
        ('pierre decalee a gauche',     [(0.45, 0.10, 100, low, 13)], (0.45, 0.10)),
        ('pile de 2',                   [(0.40, 0.00, 90, low, 13), (0.40, 0.00, 90, l2, 13)], (0.40, 0.00)),
        ('pile de 3',                   [(0.45, 0.00, 90, low, 13), (0.45, 0.00, 90, l2, 13),
                                         (0.45, 0.00, 90, l3, 13)], (0.45, 0.00)),
        ('pile de 2, vue de 70 cm',     [(0.70, 0.00, 90, low, 13), (0.70, 0.00, 90, l2, 13)], (0.70, 0.00)),
        ('deux pierres en profondeur',  [(0.40, -0.04, 90, low, 13), (0.80, 0.06, 90, low, 13)], (0.40, -0.04)),
        ('deux pierres en biais',       [(0.45, -0.09, 70, low, 13), (0.55, 0.10, 110, low, 13)], (0.45, -0.09)),
        ('proche en biais + pile loin', [(0.40, -0.06, 80, low, 13), (0.75, 0.08, 90, low, 13),
                                         (0.75, 0.08, 90, l2, 13)], (0.40, -0.06)),
        ('parasite id 7 seul',          [(0.40, 0.00, 90, low, 7)], None),
        ('parasite loin, pierre pres',  [(0.45, -0.09, 70, low, 13), (0.60, 0.11, 110, low, 7)], (0.45, -0.09)),
        ('parasite PRES, pierre loin',  [(0.45, -0.09, 70, low, 7), (0.60, 0.11, 110, low, 13)], (0.60, 0.11)),
    ]


def cmd_stone(args):
    """Rend des pierres et verifie que stone.py choisit la bonne."""
    spec = stone.load(params.bringup_share() / 'config' / 'stone.yaml')
    k, dist, size, pose, hw = bench_camera(args.revision, args.tilt)
    cam = (k, dist, size, pose)
    limits = stone.Limits()
    standoff = args.gap + hw['footprint']['front_x']
    camera_xy = pose[:3, 3]
    print(f"  camera a {pose[2, 3] * 1000:.0f} mm, inclinee de {args.tilt:.1f} deg, "
          f"arret vise a {args.gap * 100:.0f} cm de la face")

    if args.scene:
        cases = [('scene donnee', [(x, y, yaw, spec.stacked(level), spec.marker_id)
                                   for x, y, yaw, level in args.scene], None)]
    else:
        cases = battery(spec)

    print(f"\n  {'scenario':28s} {'vus':>4} {'retenus':>8} {'rejets':22s} "
          f"{'cible':>13} {'ecart':>8}  pose d arret")
    wrong = 0
    for label, stones, expected in cases:
        scene = render_stones(stones, spec, cam, args.noise)
        tags = stone.assess(detect_stones(scene, spec, cam), spec, camera_xy, limits)
        chosen = stone.select(tags, limits)
        motifs = ','.join(sorted({t.reason for t in tags if t.reason})) or '-'
        seen, kept = len(tags), sum(t.ok for t in tags)
        if chosen is None:
            bad = expected is not None
            print(f"  {label:28s} {seen:4d} {kept:8d} {motifs:22s} {'aucune':>13}"
                  f"{'   <-- CIBLE MANQUEE' if bad else ''}")
        else:
            target = stone.dock_target(chosen, standoff)
            arret = f"({target.x:.3f},{target.y:+.3f},{math.degrees(target.yaw):+6.1f} deg)"
            if expected is None:
                bad = not args.scene      # une scene donnee n attend aucune cible precise
                print(f"  {label:28s} {seen:4d} {kept:8d} {motifs:22s} "
                      f"({chosen.centre[0]:.2f},{chosen.centre[1]:+.2f}) {'':>8}  {arret}"
                      f"{'   <-- FAUX POSITIF' if bad else ''}")
            else:
                err = math.dist(chosen.centre[:2], expected) - spec.size[1] / 2
                bad = abs(err) > 0.010
                print(f"  {label:28s} {seen:4d} {kept:8d} {motifs:22s} "
                      f"({expected[0]:.2f},{expected[1]:+.2f}) {err * 1000:+6.1f}mm  {arret}"
                      f"{'   <-- MAUVAISE PIERRE' if bad else ''}")
        wrong += bad
        if args.out:
            cv2.imwrite(f"{args.out}/stone_{label.replace(' ', '_').replace(',', '')}.png", scene)
    print(f"\n  {len(cases) - wrong}/{len(cases)} scenarios corrects"
          + (f", rendus ecrits dans {args.out}" if args.out else ''))
    return 1 if wrong else 0


def cmd_stone_range(args):
    """Enveloppe de visibilite : a quel ecart le marqueur tient-il dans l'image ?

    C'est l'inclinaison du support qui decide, et elle seule : au-dela de la
    demi-ouverture verticale (20.4 deg), le haut de l'image passe sous la
    hauteur du marqueur et la portee s'effondre.
    """
    spec = stone.load(params.bringup_share() / 'config' / 'stone.yaml')
    print(f"  ecart face avant -> face de la pierre, marqueur de "
          f"{spec.marker_size * 1000:.0f} mm centre a {spec.lying * 1000:.0f} mm\n")
    print(f"  {'tilt':>6}  " + ' '.join(f'{g * 100:5.0f}' for g in args.gaps) + '   cm')
    for tilt in args.tilts:
        k, dist, size, pose, hw = bench_camera(args.revision, tilt)
        cam = (k, dist, size, pose)
        row = []
        for gap in args.gaps:
            x = gap + hw['footprint']['front_x'] + spec.size[1] / 2
            scene = render_stones([(x, 0.0, 90.0, spec.lying, spec.marker_id)], spec, cam, args.noise)
            found = detect_stones(scene, spec, cam)
            row.append(f"{'  .  ' if not found else f'{len(found):5d}'}")
        print(f"  {tilt:5.1f}d  " + ' '.join(row))
    print("\n  nombre de detections ; '.' = marqueur non decode, 1 = attendu")
    return 0


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
    pub_img = node.create_publisher(Image, '/camera/image_raw', 5)
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
        img.header.frame_id = info.header.frame_id = 'camera_optical_frame'
        pub_img.publish(img)
        pub_info.publish(info)
        time.sleep(1 / 15.0)
    rclpy.shutdown()


def cmd_fake_stones(args):
    """Detections qu'une vraie camera ferait de pierres posees dans la scene.

    Pour la simulation tant qu'elle n'a pas de pierres texturees : elles sont
    fixes dans odom (qui vaut le monde tant que le robot simule ne glisse pas),
    et chaque face laterale est publiee sur /aruco_detections si elle regarde la
    camera et tient entiere dans l'image calibree. Cela exerce toute la chaine
    du docking : stone_detector, docking_server, gestionnaire de modes -- y
    compris les piles et les pierres voisines, que le rendu Gazebo mettra plus
    longtemps a produire. Les detections sont publiees avec le retard mesure sur
    le robot (--latency), pour que la compensation de docking_server soit
    exercee comme en vrai.
    """
    import rclpy
    import tf2_ros
    import yaml
    from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
    from geometry_msgs.msg import Point, Pose, Quaternion
    from rclpy.node import Node

    spec = stone.load(params.bringup_share() / 'config' / 'stone.yaml')
    calib = yaml.safe_load(
        (params.description_share() / 'config' / 'camera_v1.yaml').read_text())
    k = np.array(calib['camera_matrix']['data']).reshape(3, 3)
    w, h = calib['image_width'], calib['image_height']
    half = spec.marker_size / 2
    corners = np.array([[-half, half, 0, 1], [half, half, 0, 1],
                        [half, -half, 0, 1], [-half, -half, 0, 1]]).T

    # Une pose par FACE laterale : les deux grandes faces d'une pierre sont
    # perpendiculaires a son grand axe, et portent le meme identifiant.
    faces = []
    for (x, y, yaw, level) in args.stones:
        for side in (1, -1):
            normal = math.radians(yaw) + side * math.pi / 2
            centre = np.array([x, y]) + (spec.size[1] / 2) * np.array(
                [math.cos(normal), math.sin(normal)])
            faces.append(stone.face_pose(centre, normal, spec.stacked(level)))

    rclpy.init()
    node = Node('aruco_bench_fake_stones')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    pub = node.create_publisher(ArucoDetection, '/aruco_detections', 10)
    frame = 'camera_optical_frame'
    seen_before = None
    pending = deque()       # (instant de l'image, detection) en attente de la latence

    def tick():
        nonlocal seen_before
        try:
            t = buf.lookup_transform(frame, 'odom', rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        tr, q = t.transform.translation, t.transform.rotation
        cam_from_odom = stone.matrix([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w])
        msg = ArucoDetection()
        msg.header.stamp = t.header.stamp
        msg.header.frame_id = frame
        for pose in faces:
            m = cam_from_odom @ pose
            pts = m @ corners
            if np.any(pts[2] < 0.05) or m[:3, 2] @ -m[:3, 3] <= 0.3 * np.linalg.norm(m[:3, 3]):
                continue            # derriere la camera, ou face vue a plus de ~72 degres
            uv = (k @ pts[:3])[:2] / pts[2]
            if np.any(uv[0] < 0) or np.any(uv[0] >= w) or np.any(uv[1] < 0) or np.any(uv[1] >= h):
                continue            # marqueur coupe par le bord de l'image
            qx, qy, qz, qw = stone.quaternion(m[:3, :3])
            msg.markers.append(MarkerPose(marker_id=spec.marker_id, pose=Pose(
                position=Point(x=m[0, 3], y=m[1, 3], z=m[2, 3]),
                orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))))
        # Les identifiants sont tous les memes : c'est le NOMBRE de faces vues
        # qui renseigne, et lui seul.
        count = len(msg.markers)
        if count != seen_before:
            print(f'faces vues : {count}', flush=True)
            seen_before = count
        now = rclpy.time.Time.from_msg(t.header.stamp).nanoseconds
        pending.append((now, msg))
        while pending and now - pending[0][0] >= args.latency * 1e9:
            pub.publish(pending.popleft()[1])

    node.create_timer(1 / 15.0, tick)
    for (x, y, yaw, level) in args.stones:
        print(f'pierre en ({x:.2f}, {y:.2f}) dans odom, cap {yaw:.0f} deg, '
              f'niveau {level} (marqueur a {spec.stacked(level) * 1000:.0f} mm)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


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

    def scene(text):
        """'x,y,cap[,niveau]' : une pierre posee, cap de son GRAND AXE en degres
        (90 = une grande face en face du robot), niveau 1 par defaut (celle du
        bas d'une pile)."""
        parts = text.split(',')
        if not 3 <= len(parts) <= 4:
            raise argparse.ArgumentTypeError(f"'{text}' : attendu x,y,cap[,niveau]")
        x, y, yaw = (float(v) for v in parts[:3])
        return x, y, yaw, int(parts[3]) if len(parts) == 4 else 1

    c = sub.add_parser('stone', help='rend des pierres et verifie le choix de la cible')
    c.add_argument('scene', nargs='*', type=scene,
                   help='x,y,cap[,niveau] ; sans argument, la batterie de reference')
    c.add_argument('--tilt', type=float, default=15.0, help='inclinaison du support, degres')
    c.add_argument('--gap', type=float, default=0.10, help='ecart vise a l arret, m')
    c.add_argument('--noise', type=float, default=1.5, help='bruit gaussien de l image')
    c.add_argument('--revision', default='v1')
    c.add_argument('--out', help='repertoire ou ecrire les rendus')
    c.set_defaults(func=cmd_stone)

    r = sub.add_parser('stone-range', help='enveloppe de visibilite selon l inclinaison')
    r.add_argument('--tilts', type=float, nargs='+',
                   default=[23.5, 20.0, 17.0, 15.0, 12.0, 10.0])
    r.add_argument('--gaps', type=float, nargs='+',
                   default=[0.10, 0.125, 0.15, 0.20, 0.30, 0.50, 0.70, 0.90])
    r.add_argument('--noise', type=float, default=1.5)
    r.add_argument('--revision', default='v1')
    r.set_defaults(func=cmd_stone_range)

    f = sub.add_parser('fake-stones', help='SIMULATION : detections de pierres posees dans odom')
    f.add_argument('stones', nargs='+', type=scene, help='x,y,cap[,niveau] par pierre')
    f.add_argument('--latency', type=float, default=0.35, help='retard des detections, s')
    f.set_defaults(func=cmd_fake_stones)

    args = ap.parse_args()
    return args.func(args) or 0


if __name__ == '__main__':
    sys.exit(main())
