#!/usr/bin/env python3
"""Banc d'essai de la detection ArUco, avec ou sans robot.

    aruco_bench.py watch [duree]            # ce que le detecteur voit vraiment
    aruco_bench.py synth 1 0.50             # camera de synthese : marqueur 1 a 50 cm
    aruco_bench.py sweep                    # portee et justesse sur images parfaites
    aruco_bench.py cube 0.40 35 20          # rend le cube et verifie sa geometrie
    aruco_bench.py fake-cube 0.60 0.10 30   # SIMULATION : cube a (x, y, cap) dans odom

'synth' publie /camera/image_raw et /camera/camera_info comme le ferait la camera,
ce qui permet de valider toute la chaine sans materiel : dictionnaire, taille
de marqueur, cablage des topics. Le lancer dans un terminal, 'watch' dans un
autre. 'sweep' se passe de ROS et mesure directement dans OpenCV la portee et
l'erreur de distance ; c'est lui qui a justifie l'affinement sous-pixellique
des coins (voir alidade_bringup/config/aruco.yaml).

ATTENTION : sur le robot, les distances ne valent que ce que valent les
intrinseques de camera_bench.yaml, donc la calibration a la mire et la mise au
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

# Intrinseques de synthese, volontairement independantes de camera_bench.yaml :
# les images sont generees et interpretees avec, si bien qu'un ecart mesure ne
# vient que de la localisation des coins, jamais de la calibration.
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


def load_cube():
    """Description du cube (config/cube.yaml) et repere de chaque marqueur.

    Rend, par identifiant, la rotation marqueur -> cube et le centre de la
    face. Les colonnes de la rotation sont les axes du marqueur exprimes dans
    le repere du cube, dans l'ordre x, y, z d'OpenCV.
    """
    import yaml
    from alidade_bringup.params import bringup_share
    spec = yaml.safe_load((bringup_share() / 'config' / 'cube.yaml').read_text())
    faces = {}
    for f in spec['faces']:
        z = np.array(f['normal'], float)
        y = np.array(f['up'], float)
        x = np.cross(y, z)
        rot = np.column_stack([x, y, z])
        faces[f['id']] = (rot, z * spec['edge'] / 2.0, f['nom'])
    return spec, faces


def cube_corners(spec, faces, mid):
    """Les 4 coins du marqueur `mid` dans le repere du cube, dans l'ordre
    d'OpenCV : haut-gauche, haut-droit, bas-droit, bas-gauche."""
    rot, centre, _ = faces[mid]
    h = spec['marker_size'] / 2.0
    local = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)
    return (rot @ local.T).T + centre


def cmd_cube(args):
    spec, faces = load_cube()
    dic = dictionary()
    k = np.array([[FX, 0, CX], [0, FX, CY], [0, 0, 1]], float)
    dist = np.zeros(5)

    # Pose imposee au cube : pose devant la camera, sa face avant tournee vers
    # elle, puis pivote. Changement d'axes du repere cube (x avant, y gauche,
    # z haut) vers le repere optique (x droite, y bas, z avant) : l'avant du
    # cube regarde la camera (x -> -z), sa gauche apparait a droite de l'image
    # (y -> x), son haut est en haut (z -> -y). A lacet nul, on voit donc la
    # face 2, comme le robot la verra en approche.
    base = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]])
    cy, sy = math.cos(math.radians(args.yaw)), math.sin(math.radians(args.yaw))
    cp, sp = math.cos(math.radians(args.pitch)), math.sin(math.radians(args.pitch))
    spin = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ \
        np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rot_true = base @ spin
    t_true = np.array([0.0, 0.0, args.distance])

    scene = np.full((H, W), 210, np.uint8)
    vues = []
    for mid, (rot_f, centre, nom) in sorted(faces.items()):
        normale_cam = rot_true @ (rot_f @ np.array([0.0, 0.0, 1.0]))
        centre_cam = rot_true @ centre + t_true
        if float(normale_cam @ centre_cam) >= 0:      # face tournee a l'oppose
            continue
        vues.append((mid, nom))
        # La FACE entiere est plaquee d'un coup : le marqueur et sa marge
        # blanche warpes ensemble, masques par le contour de la face. Masquer
        # au contour du marqueur rognait son lisere noir exterieur d'un pixel,
        # et un lisere rompu rend le marqueur indecodable -- panne bien plus
        # difficile a lire que la vraie.
        marge = int(round(200 * (spec['edge'] - spec['marker_size'])
                          / (2 * spec['marker_size'])))
        tag = cv2.copyMakeBorder(marker_image(dic, mid), marge, marge, marge, marge,
                                 cv2.BORDER_CONSTANT, value=255)
        n = tag.shape[0] - 1
        src = np.float32([[0, 0], [n, 0], [n, n], [0, n]])
        h = spec['edge'] / 2.0
        face_local = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)
        face_pts = (rot_f @ face_local.T).T + centre
        dst = cv2.projectPoints(face_pts, rot_true, t_true, k, dist)[0] \
            .reshape(-1, 2).astype(np.float32)
        warped = cv2.warpPerspective(tag, cv2.getPerspectiveTransform(src, dst), (W, H),
                                     borderValue=255, flags=cv2.INTER_LINEAR)
        mask = np.zeros((H, W), np.uint8)
        cv2.fillConvexPoly(mask, dst.astype(np.int32), 255)
        scene[mask > 0] = warped[mask > 0]

    print(f"  cube a {args.distance:.2f} m, lacet {args.yaw:+.0f} deg, tangage {args.pitch:+.0f} deg")
    print(f"  faces tournees vers la camera : {', '.join(f'{m} ({n})' for m, n in vues)}")

    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        p = cv2.aruco.DetectorParameters_create()
    else:
        p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(scene, dic, parameters=p)
    trouves = sorted(ids.flatten().tolist()) if ids is not None else []
    print(f"  marqueurs decodes            : {trouves if trouves else 'AUCUN'}")

    out = '/tmp/cube_rendu.png'
    cv2.imwrite(out, scene)
    print(f"  rendu ecrit dans {out}")

    if not trouves:
        return 1
    # PnP conjoint sur tous les coins vus : c'est lui qui met la table de
    # geometrie a l'epreuve. Une face mal orientee ferait exploser l'erreur.
    obj, img = [], []
    for mid, quad in zip(ids.flatten().tolist(), corners):
        obj.append(cube_corners(spec, faces, mid))
        img.append(quad.reshape(-1, 2))
    ok, rvec, tvec = cv2.solvePnP(np.vstack(obj).astype(np.float64),
                                  np.vstack(img).astype(np.float64), k, dist,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        print("  resolution de pose impossible")
        return 1
    err_pos = float(np.linalg.norm(tvec.ravel() - t_true)) * 1000.0
    rot_est = cv2.Rodrigues(rvec)[0]
    err_ang = math.degrees(math.acos(
        max(-1.0, min(1.0, (np.trace(rot_est.T @ rot_true) - 1.0) / 2.0))))
    print(f"  pose du cube retrouvee       : erreur {err_pos:.1f} mm et {err_ang:.2f} deg")
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


def cmd_fake_cube(args):
    """Detections qu'une vraie camera ferait d'un cube pose dans la scene.

    Pour la simulation, qui n'a pas de cube ArUco : le cube est fixe dans odom
    (qui vaut le monde tant que le robot simule ne glisse pas), et chaque face
    est publiee sur /aruco_detections si elle regarde la camera et tient
    entiere dans l'image calibree. Cela exerce toute la chaine du docking :
    cube_detector, cube_docking, gestionnaire de modes. Les detections sont
    publiees avec le retard mesure sur le robot (--latency), pour que la
    compensation de cube_docking soit exercee comme en vrai.
    """
    import rclpy
    import tf2_ros
    import yaml
    from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
    from geometry_msgs.msg import Point, Pose, Quaternion
    from rclpy.node import Node

    from alidade_bringup import cube, params

    edge, faces = cube.load(params.bringup_share() / 'config' / 'cube.yaml')
    calib = yaml.safe_load((params.description_share() / 'config' / 'camera_bench.yaml').read_text())
    k = np.array(calib['camera_matrix']['data']).reshape(3, 3)
    w, h = calib['image_width'], calib['image_height']
    half = SIDE / 2
    corners = np.array([[-half, half, 0, 1], [half, half, 0, 1], [half, -half, 0, 1], [-half, -half, 0, 1]]).T
    yaw = math.radians(args.yaw)
    in_odom = np.eye(4)
    in_odom[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    in_odom[:3, 3] = [args.x, args.y, edge / 2]

    rclpy.init()
    node = Node('aruco_bench_fake_cube')
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
        cam_from_odom = cube.matrix([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w])
        msg = ArucoDetection()
        msg.header.stamp = t.header.stamp
        msg.header.frame_id = frame
        for fid, face in faces.items():
            m = cam_from_odom @ in_odom @ face.pose
            pts = m @ corners
            if np.any(pts[2] < 0.05) or m[:3, 2] @ -m[:3, 3] <= 0.3 * np.linalg.norm(m[:3, 3]):
                continue            # derriere la camera, ou face vue a plus de ~72 degres
            uv = (k @ pts[:3])[:2] / pts[2]
            if np.any(uv[0] < 0) or np.any(uv[0] >= w) or np.any(uv[1] < 0) or np.any(uv[1] >= h):
                continue            # marqueur coupe par le bord de l'image
            qx, qy, qz, qw = cube.quaternion(m[:3, :3])
            msg.markers.append(MarkerPose(marker_id=fid, pose=Pose(
                position=Point(x=m[0, 3], y=m[1, 3], z=m[2, 3]),
                orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))))
        ids = sorted(mk.marker_id for mk in msg.markers)
        if ids != seen_before:
            print(f"faces vues : {ids or 'aucune'}", flush=True)
            seen_before = ids
        now = rclpy.time.Time.from_msg(t.header.stamp).nanoseconds
        pending.append((now, msg))
        while pending and now - pending[0][0] >= args.latency * 1e9:
            pub.publish(pending.popleft()[1])

    node.create_timer(1 / 15.0, tick)
    print(f'cube en ({args.x:.2f}, {args.y:.2f}) dans odom, cap {args.yaw:.0f} deg')
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

    c = sub.add_parser('cube', help='rend le cube et verifie sa geometrie')
    c.add_argument('distance', nargs='?', type=float, default=0.40)
    c.add_argument('yaw', nargs='?', type=float, default=35.0)
    c.add_argument('pitch', nargs='?', type=float, default=20.0)
    c.set_defaults(func=cmd_cube)

    f = sub.add_parser('fake-cube', help='SIMULATION : detections d un cube pose dans odom')
    f.add_argument('x', type=float)
    f.add_argument('y', type=float)
    f.add_argument('yaw', nargs='?', type=float, default=0.0, help='cap du cube, degres')
    f.add_argument('--latency', type=float, default=0.35, help='retard des detections, s')
    f.set_defaults(func=cmd_fake_cube)

    args = ap.parse_args()
    return args.func(args) or 0


if __name__ == '__main__':
    sys.exit(main())
