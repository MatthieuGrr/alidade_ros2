#!/usr/bin/env python3
"""Ou tombe le 0 degre du LiDAR, vu du robot ? Controle de banc, deux minutes.

    ros2 run alidade_bringup lidar_zero.py                 # objet droit devant
    ros2 run alidade_bringup lidar_zero.py --azimut 90     # objet a gauche
    ros2 run alidade_bringup lidar_zero.py --azimut -135   # arriere-droit

Posez un objet isole (une boite, un pied de chaise) a l'azimut annonce, entre
30 cm et 1.50 m du robot, et DEGAGEZ le reste du tour : le script prend le
retour le plus proche et dit a quel angle le ROBOT le place.

Pourquoi ce controle existe : les pilotes Slamtec publient theta_ros = pi -
theta_brut, la ou la conversion juste est theta_ros = -theta_brut. L'ecart est
d'exactement un demi-tour, il est invisible autrement (un scan retourne reste
un scan plausible), et il fait cartographier la piece a l'envers.

sllidar_node porte les memes lignes que l'ancien rplidar_ros, mais il n'a plus
de parametre `flip_x_axis` : la correction vit desormais dans
`lidar.mount_rpy` de alidade_description/config/alidade_v<n>.yaml, donc dans
TF et non dans /scan.

C'est pourquoi ce script TRANSFORME le point mesure dans `base_link` avant de
juger. Lire /scan directement ne dirait rien d'utile : la correction n'y est
pas. Au passage la transformation tient compte du deport du capteur, 10 cm en
arriere du centre, qui decale de plusieurs degres l'azimut d'un objet proche.

A refaire apres tout changement de pilote, de capteur, de support ou de
mount_rpy.
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def applique(tf, x: float, y: float):
    """Transforme le point (x, y, 0) par un geometry_msgs/TransformStamped.

    Rotation par le quaternion puis translation. On n'importe pas
    tf2_geometry_msgs : il tire PyKDL, absent du Pi, pour trois lignes.
    """
    q = tf.transform.rotation
    t = tf.transform.translation
    # Lignes de la matrice de rotation utiles a un point du plan z = 0.
    xr = (1 - 2 * (q.y * q.y + q.z * q.z)) * x + 2 * (q.x * q.y - q.z * q.w) * y
    yr = 2 * (q.x * q.y + q.z * q.w) * x + (1 - 2 * (q.x * q.x + q.z * q.z)) * y
    return xr + t.x, yr + t.y


class Zero(Node):
    def __init__(self, attendu: float, tours: int, repere: str):
        super().__init__('lidar_zero')
        self.attendu, self.restants, self.mesures = attendu, tours, []
        self.repere = repere
        self.tampon = Buffer()
        self.ecoute = TransformListener(self.tampon, self)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan):
        candidats = [(r, scan.angle_min + i * scan.angle_increment)
                     for i, r in enumerate(scan.ranges)
                     if math.isfinite(r) and 0.30 <= r <= 1.50]
        if not candidats:
            self.get_logger().warn('aucun retour entre 30 cm et 1.50 m : objet trop loin ou absent')
            return
        try:
            tf = self.tampon.lookup_transform(self.repere, scan.header.frame_id,
                                              rclpy.time.Time())
        except Exception as e:
            # Sans TF on ne peut rien conclure : le demi-tour vit justement la.
            self.get_logger().warn(
                f'{self.repere} <- {scan.header.frame_id} indisponible ({e}). '
                'robot_state_publisher tourne-t-il ?')
            return
        r, a = min(candidats)
        x, y = applique(tf, r * math.cos(a), r * math.sin(a))
        self.mesures.append((math.hypot(x, y), math.atan2(y, x), a))
        self.restants -= 1
        if self.restants <= 0:
            self.rendre(scan)
            raise SystemExit(0)

    def rendre(self, scan):
        med = sorted(a for _, a, _ in self.mesures)[len(self.mesures) // 2]
        brut = sorted(b for _, _, b in self.mesures)[len(self.mesures) // 2]
        dist = sorted(r for r, _, _ in self.mesures)[len(self.mesures) // 2]
        ecart = wrap(med - self.attendu)
        print(f"\n  objet le plus proche : {dist * 100:.1f} cm du centre du robot")
        print(f"  azimut dans {self.repere:<12}: {math.degrees(med):+.1f} deg")
        print(f"  azimut attendu       : {math.degrees(self.attendu):+.1f} deg")
        print(f"  ECART                : {math.degrees(ecart):+.1f} deg")
        print(f"  (dans {scan.header.frame_id}, avant TF : "
              f"{math.degrees(brut):+.1f} deg)")
        print(f"  champ publie         : {math.degrees(scan.angle_min):+.1f} a "
              f"{math.degrees(scan.angle_max):+.1f} deg, {len(scan.ranges)} points\n")
        if abs(math.degrees(ecart)) < 8.0:
            print("  -> le 0 degre est au bon endroit, rien a corriger.")
        elif abs(abs(math.degrees(ecart)) - 180.0) < 15.0:
            print("  -> DEMI-TOUR. Basculer `lidar.mount_rpy` entre [0,0,0] et")
            print("     [0,0,pi] dans alidade_description/config/alidade_v<n>.yaml,")
            print("     puis recompiler alidade_description.")
            print("     sllidar_node n'a PAS de `flip_x_axis` : ne pas en chercher un,")
            print("     et ne pas corriger a deux endroits (ils se compensent).")
        else:
            print("  -> ni bon, ni demi-tour. Un objet pose a vue, sans mesurer, se")
            print("     place couramment a 10-20 deg de l'azimut annonce : c'est la")
            print("     premiere chose a soupconner. Au-dela, verifier qu'il est bien")
            print("     SEUL dans la plage 30 cm - 1.50 m (un pied de table, un mur")
            print("     proche ou le bord d'une porte font de meilleurs candidats que")
            print("     l'objet vise), puis le support du capteur.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--azimut', type=float, default=0.0,
                    help='azimut reel de l objet, en degres (0 = droit devant, 90 = a gauche)')
    ap.add_argument('--tours', type=int, default=10, help='scans a moyenner')
    ap.add_argument('--repere', default='base_link',
                    help='repere ou juger l azimut (defaut base_link)')
    args = ap.parse_args()
    rclpy.init()
    node = Zero(math.radians(args.azimut), args.tours, args.repere)
    print(f'  posez l objet a {args.azimut:+.0f} deg, entre 30 cm et 1.50 m, le reste degage...')
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
