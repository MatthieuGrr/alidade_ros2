#!/usr/bin/env python3
"""Calibration de l'odometrie, le LiDAR contre la scene pour juge.

    odom_calib.py rotate [tours] [w]      # echelle en rotation (tours complets)
    odom_calib.py straight [metres] [v]   # cap apres une ligne droite (ICP)

A refaire apres tout changement de pneus, de plancher, de firmware moteur ou
d'IMU.

SECURITE. L'outil publie sur /cmd_vel_nav, donc EN AMONT de la chaine nav2 :
velocity_smoother, puis collision_monitor, qui bloque tout sous 22 cm. Il est
protege comme la navigation l'est.

Ce n'etait pas le cas avant le 29 septembre 2026, ou il publiait directement
sur /diff_drive_controller/cmd_vel : un robot a touche un mur ce jour-la, la
verification de degagement ayant ete faite AVANT le lancement et dans la meme
commande, donc hors d'etat d'empecher quoi que ce soit.

Ses propres garde-fous (`libre`, MARGE_AVANT, RAYON_CIRCONSCRIT) restent utiles
et ne font pas double emploi : ils REFUSENT avant de rouler quand la place
manque, au lieu de laisser le moniteur bloquer a mi-parcours et rendre une
mesure tronquee. Ils mesurent depuis base_link, en passant par TF -- voir
Bench.pose_laser, et ne jamais y remettre d'angle ni de deport en dur.

Trois sources sont comparees a chaque essai : l'odometrie ROUES
(/diff_drive_controller/odom), l'integrale du GYROMETRE et l'odometrie
FUSIONNEE publiee par gyro_odom.py (/odom).

Ni l'IMU ni slam_toolbox ne servent de reference : c'est le LiDAR contre la
scene, de deux facons selon le mouvement.
  rotate   : N tours COMPLETS sur place, profils de scan avant/apres correles
             circulairement. Les tours partiels seraient biaises, le LiDAR
             etant excentre (152 mm en arriere de base_link sur la v2 ; la
             valeur exacte vient de TF, pas d'ici).
  straight : ICP 2D point a point entre les nuages avant/apres, initialise par
             l'odometrie. La correlation de profils ne vaut que pour une
             rotation pure, d'ou l'ICP des qu'il y a translation.

Resultats a reporter dans alidade_<revision>.yaml : kinematics pour les roues,
imu.gyro_scale pour le gyromètre. Mesure du 12 sept. 2026 : roues 0.9975 a
1.0061, gyromètre 1.029 (d'ou gyro_scale 0.9718), fusion 0.9989.

CAMPAGNE DU 7 OCTOBRE 2026, apres le passage a sllidar_ros2. Trois rotations de
2 tours (deux anti-horaire, une horaire) : roues 1.0026 / 1.0036 / 1.0016,
gyromètre 1.0285 / 1.0285 / 1.0331, fusion 0.9997 / 0.9992 / 1.0045. Une ligne
droite de 1 m a 0.15 m/s, 330 appariements : distance roues +0.12 %, cap +0.66
deg (fusion +0.58). RIEN N'A ETE CHANGE dans la revision, et c'est le resultat :
l'ecart des roues en rotation, +0.26 % de biais systematique, est du MEME ORDRE
que la resolution du juge -- le residu du recalage est quantifie au degre et
s'etale sur +/-1.5 deg de 720, soit +/-0.2 %. Corriger wheel_separation sur
cette base serait courir apres le bruit du banc. L'echelle du gyromètre, elle,
est confirmee a 0.05 % pres.

Degager un metre autour du robot.
"""
import argparse
import math
import statistics
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener

from alidade_bringup import odometry

BINS = 360


# Marges de securite. Le chassis v2 deborde de 55 mm devant base_footprint et
# son rayon circonscrit vaut 235 mm (coins arriere du plateau) : en pivotant
# sur place il balaie donc ce rayon dans TOUTES les directions.
MARGE_AVANT = 0.35            # m, distance minimale devant pendant une ligne droite
# En deca, le recalage n'a pas de quoi trancher et rend un chiffre plausible mais
# faux. Les bons essais du 29 sept. 2026 en avaient 326 et 329 ; les mauvais, 175
# et 182, pour des erreurs annoncees de 18 et 34 % que le robot ne faisait pas.
APPARIEMENTS_MIN = 250
RAYON_CIRCONSCRIT = 0.235     # m, encombrement balaye en rotation


def spin(node, dur):
    end = time.time() + dur
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def integrate(samples, sens=1.0, bias=0.0):
    """Integrale trapezoidale d'une serie (instant, vitesse angulaire)."""
    return sum(sens * (w1 + w2 - 2 * bias) / 2 * (t2 - t1)
               for (t1, w1), (t2, w2) in zip(samples, samples[1:]) if 0 < t2 - t1 < 0.1)


class Bench(Node):
    """Abonnements communs aux deux mesures, et la commande de vitesse."""

    def __init__(self):
        super().__init__('odom_calib')
        self.wheel = None       # (x, y, cap) de l'odometrie roues
        self.fused = None       # cap de /odom
        self.scans = []
        self.gyro = []
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.on_wheel, 20)
        self.create_subscription(Odometry, '/odom', self.on_fused, 20)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scans.append, qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.tampon = Buffer()
        self.ecoute = TransformListener(self.tampon, self)
        self._laser = None      # (x, y, lacet) du capteur dans base_link, mis en cache
        self._rot_imu = None    # rotation IMU -> base_link, mise en cache

    def on_wheel(self, m):
        p = m.pose.pose
        self.wheel = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_fused(self, m):
        self.fused = yaw_of(m.pose.pose.orientation)

    def rot_imu(self, frame: str):
        """Rotation IMU -> base_link, lue dans TF une fois (joint fixe).

        L'IMU de ce robot est montee TETE EN BAS : son z pointe vers le sol, et
        sa vitesse angulaire z brute a donc le signe OPPOSE a celle du robot.
        Prendre `angular_velocity.z` tel quel -- ce que faisait ce script
        jusqu'au 7 octobre 2026 -- rendait un rapport gyrometre/vrai de -1.0286
        la ou gyro_odom.py, qui passe par TF, trouvait la fusion a 0.9997. La
        magnitude etait bonne, le signe faux : une erreur qui ne se voit pas si
        on ne regarde que la valeur absolue.
        """
        if self._rot_imu is None:
            try:
                tf = self.tampon.lookup_transform('base_link', frame, rclpy.time.Time())
            except Exception:
                return None
            q = tf.transform.rotation
            self._rot_imu = odometry.rotation_from_quaternion(q.x, q.y, q.z, q.w)
            sens = ('a l endroit' if self._rot_imu[2][2] > 0
                    else 'A L ENVERS (z du capteur vers le bas)')
            print(f'IMU {frame} : {sens}, wz -> {self._rot_imu[2][2]:+.3f}')
        return self._rot_imu

    def on_imu(self, m):
        rot = self.rot_imu(m.header.frame_id)
        if rot is None:
            return
        t = rclpy.time.Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        g = m.angular_velocity
        self.gyro.append((t, odometry.yaw_rate(rot, (g.x, g.y, g.z))))

    def ready(self):
        # On attend AUSSI un echantillon de gyrometre : il n'arrive qu'une fois
        # la rotation IMU -> base_link disponible dans TF, et sans elle les
        # chiffres du gyrometre seraient faux de signe sans le dire.
        fin = time.time() + 20.0
        while (self.wheel is None or self.fused is None or len(self.scans) < 3
               or not self.gyro):
            if time.time() > fin:
                manque = [n for n, ok in (('odometrie roues', self.wheel is not None),
                                          ('/odom', self.fused is not None),
                                          ('/scan', len(self.scans) >= 3),
                                          ('gyrometre via TF', bool(self.gyro))) if not ok]
                raise SystemExit('REFUS : rien recu en 20 s pour ' + ', '.join(manque))
            rclpy.spin_once(self, timeout_sec=0.1)

    def drive(self, v=0.0, w=0.0):
        self.cmd.publish(Twist(linear=Vector3(x=v), angular=Vector3(z=w)))

    def stop(self):
        for _ in range(6):
            self.drive(); spin(self, 0.05)
        spin(self, 1.5)

    def pose_laser(self):
        """Pose du capteur dans base_link : (x, y, lacet), lue dans TF une fois.

        RIEN ICI N'EST EN DUR, et c'est essentiel. Le pilote Slamtec publie
        theta_ros = pi - theta_brut (sllidar_node.cpp, lignes 218-222), soit un
        DEMI-TOUR par rapport au repere du robot. Cette correction ne vit plus
        dans le pilote -- sllidar_node n'a pas de `flip_x_axis` -- mais dans
        `lidar.mount_rpy` de alidade_<revision>.yaml, donc dans la transformee
        base_link -> laser. Tout ce qui lit /scan sans passer par TF voit donc
        les angles RETOURNES.

        Ce script l'a appris a ses depens : son cone de degagement avant,
        calcule sur les angles bruts, regardait en realite VERS L'ARRIERE, et
        `cloud()` compensait de son cote par un `+180` ecrit en dur avec un
        deport `laser_x` reste de la v1. Deux conventions contradictoires dans
        le meme fichier, sur l'outil qui fait avancer le robot sans le
        collision_monitor.
        """
        if self._laser is None:
            fin = time.time() + 5.0
            while time.time() < fin:
                try:
                    tf = self.tampon.lookup_transform('base_link', 'laser',
                                                      rclpy.time.Time())
                except Exception:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    continue
                t = tf.transform.translation
                self._laser = (t.x, t.y, yaw_of(tf.transform.rotation))
                break
            else:
                # Pas de repli : sans cette transformee les garde-fous sont faux,
                # et un garde-fou faux est pire que pas de garde-fou.
                raise SystemExit(
                    'REFUS : base_link <- laser absente de TF. robot_state_publisher '
                    'tourne-t-il ? Sans elle les distances de securite sont fausses.')
        return self._laser

    def points(self, scans=None, rmin=None, rmax=None):
        """Points des scans exprimes dans base_link."""
        px, py, lacet = self.pose_laser()
        pts = []
        for sc in (scans if scans is not None else self.scans[-1:]):
            bas = sc.range_min if rmin is None else rmin
            haut = sc.range_max if rmax is None else rmax
            for i, r in enumerate(sc.ranges):
                if not math.isfinite(r) or not (bas <= r <= haut):
                    continue
                a = sc.angle_min + i * sc.angle_increment + lacet
                pts.append((px + r * math.cos(a), py + r * math.sin(a)))
        return pts

    def libre(self, demi_angle=None):
        """Distance au plus proche obstacle, mesuree DEPUIS base_link.

        `demi_angle` a None regarde tout autour (pour les rotations sur place) ;
        une valeur en degres restreint au cone centre sur +x (pour les lignes
        droites), car un obstacle derriere n'est pas un danger quand on avance.

        La distance est prise depuis base_link et non depuis le capteur, parce
        que c'est a base_link que se rapportent les deux seuils : MARGE_AVANT
        et RAYON_CIRCONSCRIT decrivent l'encombrement du CHASSIS. Le capteur
        etant 152 mm en arriere du centre, les comparer a une portee mesuree
        depuis le capteur surestimait le degagement devant d'autant.
        """
        if not self.scans:
            return 0.0
        proche = float('inf')
        for x, y in self.points():
            if demi_angle is not None and abs(math.degrees(math.atan2(y, x))) > demi_angle:
                continue
            proche = min(proche, math.hypot(x, y))
        return proche

    def fresh_scans(self, seconds=7.0):
        """Six tours de LiDAR : la mediane par degre lisse le bruit du capteur."""
        self.scans.clear(); spin(self, seconds)
        return self.scans[-6:]


# --------------------------------------------------------------- rotation
def profile(bench, scans):
    """Portee mediane par degre, vue de base_link, sur plusieurs tours.

    Les azimuts sont ceux du ROBOT, pas ceux du capteur : voir Bench.pose_laser.
    Pour une correlation avant/apres, un decalage constant s'annulerait de toute
    facon -- mais deux conventions dans un meme fichier finissent par se croiser.
    """
    acc = [[] for _ in range(BINS)]
    for sc in scans:
        for x, y in bench.points([sc], rmin=0.05):
            acc[int(round(math.degrees(math.atan2(y, x)))) % BINS].append(math.hypot(x, y))
    return [statistics.median(v) if v else None for v in acc]


def circular_shift(a, b):
    """Decalage circulaire, en degres, qui superpose b sur a."""
    best, arg = None, 0
    for k in range(BINS):
        pairs = [(a[i], b[(i + k) % BINS]) for i in range(BINS)]
        pairs = [(x, y) for x, y in pairs if x and y and abs(x - y) < 1.5]
        if len(pairs) < BINS // 4:
            continue
        err = sum(abs(x - y) for x, y in pairs) / len(pairs)
        if best is None or err < best:
            best, arg = err, k
    return (arg + 180) % 360 - 180, best


def cmd_rotate(bench, args):
    sens = 1.0 if args.turns >= 0 else -1.0
    target = abs(args.turns) * 2 * math.pi
    print('profil de depart...')
    before = profile(bench, bench.fresh_scans())
    bench.gyro.clear(); spin(bench, 4.0)
    bias = statistics.fmean(w for _, w in bench.gyro)
    print(f"biais du gyromètre a l'arret : {math.degrees(bias):+.4f} deg/s "
          f"({len(bench.gyro)} echantillons)")

    turned = fused = 0.0
    prev_w, prev_f = bench.wheel[2], bench.fused
    bench.gyro.clear()
    place = bench.libre()
    if place < RAYON_CIRCONSCRIT + 0.05:
        raise SystemExit(
            f'REFUS : obstacle a {place:.2f} m, le chassis balaie '
            f'{RAYON_CIRCONSCRIT:.2f} m en pivotant sur place.')
    print(f"{abs(args.turns):g} tour(s) a {args.w} rad/s, "
          f"{'anti-horaire' if sens > 0 else 'horaire'} ({place:.2f} m de degagement)...")
    start = time.time()
    while turned < target and time.time() - start < 180:
        rclpy.spin_once(bench, timeout_sec=0.02)
        turned += sens * wrap(bench.wheel[2] - prev_w); prev_w = bench.wheel[2]
        fused += sens * wrap(bench.fused - prev_f); prev_f = bench.fused
        left = target - turned
        if bench.libre() < RAYON_CIRCONSCRIT + 0.05:
            bench.stop()
            raise SystemExit(f'ARRET : obstacle a {bench.libre():.2f} m, le chassis balaie '
                             f'{RAYON_CIRCONSCRIT:.2f} m en pivotant')
        bench.drive(w=sens * max(0.45, min(args.w, 1.5 * left)) if left > 0.02 else 0.0)
    bench.stop()
    gyro = integrate(bench.gyro, sens, bias)
    turned += sens * wrap(bench.wheel[2] - prev_w)
    fused += sens * wrap(bench.fused - prev_f)

    print('profil final...')
    residual, err = circular_shift(before, profile(bench, bench.fresh_scans()))
    truth = abs(args.turns) * 360 + sens * (-residual)
    print()
    print(f"roues       {math.degrees(turned):9.2f} deg")
    print(f"gyrometre   {math.degrees(gyro):9.2f} deg")
    print(f"/odom       {math.degrees(fused):9.2f} deg")
    print(f"LiDAR       residu {residual:+.1f} deg -> rotation vraie {truth:9.2f} deg "
          f"(ecart moyen des profils {err:.3f} m)")
    print(f"rapports    roues/vrai {math.degrees(turned)/truth:.4f}   "
          f"gyrometre/vrai {math.degrees(gyro)/truth:.4f}   fusion/vrai {math.degrees(fused)/truth:.4f}")


# --------------------------------------------------------------- ligne droite
def cloud(bench, scans):
    """Nuage 2D dans base_link, median par degre.

    Le demi-tour du capteur et son deport viennent de TF (Bench.pose_laser).
    Cette fonction portait jusqu'au 6 octobre 2026 un `+180` et un
    `laser_x=0.0456` ecrits en dur : le premier doublait la correction tant que
    le pilote la faisait, le second etait le deport de la v1 (il vaut -0.152 m
    en v2). Ne rien remettre en dur ici.
    """
    acc = {}
    for sc in scans:
        for x, y in bench.points([sc], rmin=0.1, rmax=8.0):
            acc.setdefault(int(round(math.degrees(math.atan2(y, x)))), []).append(math.hypot(x, y))
    pts = []
    for a, v in acc.items():
        r, t = statistics.median(v), math.radians(a)
        pts.append((r * math.cos(t), r * math.sin(t)))
    return np.array(pts)


def icp(src, dst, guess, iters=25, reject=0.25):
    """Transformation rigide amenant src sur dst, initialisee par `guess`.

    Point a point, plus proche voisin par force brute : 360 points par nuage,
    quelques millisecondes par iteration, et aucune dependance a installer.
    """
    dx, dy, dt = guess
    kept = 0
    for _ in range(iters):
        c, s = math.cos(dt), math.sin(dt)
        moved = np.column_stack([c * src[:, 0] - s * src[:, 1] + dx,
                                 s * src[:, 0] + c * src[:, 1] + dy])
        d = np.linalg.norm(moved[:, None, :] - dst[None, :, :], axis=2)
        j = d.argmin(axis=1)
        keep = d[np.arange(len(src)), j] < reject
        kept = int(keep.sum())
        if kept < 20:
            break
        a, b = moved[keep], dst[j[keep]]
        ca, cb = a.mean(axis=0), b.mean(axis=0)
        u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
        rot = vt.T @ u.T
        if np.linalg.det(rot) < 0:
            vt[1] *= -1
            rot = vt.T @ u.T
        step = math.atan2(rot[1, 0], rot[0, 0])
        t = cb - rot @ ca
        c2, s2 = math.cos(step), math.sin(step)
        dx, dy = c2 * dx - s2 * dy + t[0], s2 * dx + c2 * dy + t[1]
        dt += step
        if abs(step) < 1e-5 and np.linalg.norm(t) < 1e-4:
            break
    return dx, dy, dt, kept


def cmd_straight(bench, args):
    print('nuage de depart...')
    before = cloud(bench, bench.fresh_scans())
    w0, f0 = bench.wheel, bench.fused
    bench.gyro.clear()
    # REFUS avant de rouler, plutot qu'un arret a mi-parcours : on sait deja que
    # la place manque, autant le dire sans bouger.
    place = bench.libre(demi_angle=30)
    if place < args.metres + MARGE_AVANT:
        raise SystemExit(
            f'REFUS : {place:.2f} m devant, il en faut {args.metres + MARGE_AVANT:.2f} '
            f'({args.metres:g} m d essai + {MARGE_AVANT:.2f} de marge). '
            f'Reculer, tourner vers du degage, ou demander moins.')
    print(f'ligne droite de {args.metres:g} m a {args.v} m/s, cap libre '
          f'({place:.2f} m devant)...')
    start = time.time()
    while time.time() - start < 60:
        rclpy.spin_once(bench, timeout_sec=0.02)
        done = math.dist(bench.wheel[:2], w0[:2])
        if done >= args.metres - 0.005:
            break
        if bench.libre(demi_angle=30) < MARGE_AVANT:
            bench.stop()
            raise SystemExit(f'ARRET : obstacle a {bench.libre(demi_angle=30):.2f} m devant, '
                             f'moins que la marge de {MARGE_AVANT:.2f} m')
        bench.drive(v=args.v if args.metres - done > 0.15 else max(0.05, args.v / 2))
    bench.stop()
    w1, f1 = bench.wheel, bench.fused
    print('nuage final...')
    after = cloud(bench, bench.fresh_scans())

    wheel_yaw, fused_yaw = wrap(w1[2] - w0[2]), wrap(f1 - f0)
    gyro_yaw = integrate(bench.gyro)
    # Un point du nuage final, vu du robot arrive, revient dans le repere de
    # depart par la pose finale : c'est donc le deplacement lui-meme qui
    # aligne `after` sur `before`, et l'odometrie en est la premiere estimation.
    dx, dy, dt, used = icp(after, before, (w1[0] - w0[0], w1[1] - w0[1], wheel_yaw))
    print()
    print(f"roues       {math.dist(w1[:2], w0[:2])*100:6.1f} cm, cap {math.degrees(wheel_yaw):+6.2f} deg")
    print(f"gyrometre   cap {math.degrees(gyro_yaw):+6.2f} deg")
    print(f"/odom       cap {math.degrees(fused_yaw):+6.2f} deg")
    print(f"ICP (juge)  {math.hypot(dx, dy)*100:6.1f} cm, cap {math.degrees(dt):+6.2f} deg "
          f"({used} appariements)")
    print(f"erreurs de cap : roues {math.degrees(wrap(wheel_yaw - dt)):+.2f} deg, "
          f"/odom {math.degrees(wrap(fused_yaw - dt)):+.2f} deg")
    print(f"erreur de distance : roues {100*(math.dist(w1[:2], w0[:2])/math.hypot(dx, dy) - 1):+.2f} %")
    if used < APPARIEMENTS_MIN:
        print()
        print(f"*** MESURE A REJETER : {used} appariements seulement, il en faut "
              f"{APPARIEMENTS_MIN}. ***")
        print("    L'ICP n'a pas assez de structure ici et converge ailleurs. Deux essais")
        print("    dans un coin degage ont ainsi annonce +34 % et +18 % d'erreur de")
        print("    distance, la ou un recalage a 330 points en trouvait 0.4 % au meme")
        print("    endroit. Se placer dans une zone ou le LiDAR voit des murs sous tous")
        print("    les azimuts, et refaire. Un chiffre issu d'un recalage pauvre est pire")
        print("    qu'une absence de mesure : il a l'air d'en etre une.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='command', required=True)
    r = sub.add_parser('rotate', help='echelle en rotation, tours complets')
    r.add_argument('turns', nargs='?', type=float, default=2.0, help='negatif = horaire')
    r.add_argument('w', nargs='?', type=float, default=0.6)
    r.set_defaults(func=cmd_rotate)
    s = sub.add_parser('straight', help='cap et distance apres une ligne droite')
    s.add_argument('metres', nargs='?', type=float, default=1.0)
    s.add_argument('v', nargs='?', type=float, default=0.15)
    s.set_defaults(func=cmd_straight)
    args = ap.parse_args()

    rclpy.init()
    bench = Bench()
    bench.ready()
    try:
        args.func(bench, args)
    finally:
        bench.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
