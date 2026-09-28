#!/usr/bin/env python3
"""Ou tombe le 0 degre du LiDAR ? Controle de banc, deux minutes.

    ros2 run alidade_bringup lidar_zero.py            # objet droit devant
    ros2 run alidade_bringup lidar_zero.py --azimut 90  # objet a gauche

Posez un objet isole (une boite, un pied de chaise) a l'azimut annonce, entre
30 cm et 1.50 m du robot, et DEGAGEZ le reste du tour : le script prend le
retour le plus proche et dit a quel angle le capteur le place.

Pourquoi ce controle existe : le pilote rplidar_ros publie theta_ros = pi -
theta_brut, la ou la conversion juste est theta_ros = -theta_brut. L'ecart est
d'exactement un demi-tour, il est invisible autrement (un scan retourne reste
un scan plausible), et il fait cartographier la piece a l'envers. Il s'annule
par `flip_x_axis: true` dans rplidar.yaml -- ce script verifie que c'est bien
le cas sur CE robot, avec CE pilote.

A refaire apres tout changement de pilote, de capteur ou de support.
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Zero(Node):
    def __init__(self, attendu: float, tours: int):
        super().__init__('lidar_zero')
        self.attendu, self.restants, self.mesures = attendu, tours, []
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan):
        candidats = [(r, scan.angle_min + i * scan.angle_increment)
                     for i, r in enumerate(scan.ranges)
                     if math.isfinite(r) and 0.30 <= r <= 1.50]
        if not candidats:
            self.get_logger().warn('aucun retour entre 30 cm et 1.50 m : objet trop loin ou absent')
            return
        r, a = min(candidats)
        self.mesures.append((r, a))
        self.restants -= 1
        if self.restants <= 0:
            self.rendre(scan)
            raise SystemExit(0)

    def rendre(self, scan):
        angles = sorted(a for _, a in self.mesures)
        med = angles[len(angles) // 2]
        dist = sorted(r for r, _ in self.mesures)[len(self.mesures) // 2]
        ecart = wrap(med - self.attendu)
        print(f"\n  objet le plus proche : {dist * 100:.1f} cm, azimut {math.degrees(med):+.1f} deg")
        print(f"  azimut attendu       : {math.degrees(self.attendu):+.1f} deg")
        print(f"  ECART                : {math.degrees(ecart):+.1f} deg")
        print(f"  champ publie         : {math.degrees(scan.angle_min):+.1f} a "
              f"{math.degrees(scan.angle_max):+.1f} deg, {len(scan.ranges)} points\n")
        if abs(math.degrees(ecart)) < 8.0:
            print("  -> le 0 degre est au bon endroit, rien a corriger.")
        elif abs(abs(math.degrees(ecart)) - 180.0) < 15.0:
            print("  -> DEMI-TOUR. Basculer `flip_x_axis` dans config/rplidar.yaml")
            print("     (et surtout PAS remettre une rotation dans lidar.mount_rpy :")
            print("      les deux se compensent, voir les commentaires des deux fichiers).")
        else:
            print("  -> ecart inattendu. Verifier que l'objet est bien seul dans la")
            print("     plage 30 cm - 1.50 m, et que l'azimut annonce est le bon.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--azimut', type=float, default=0.0,
                    help='azimut reel de l objet, en degres (0 = droit devant, 90 = a gauche)')
    ap.add_argument('--tours', type=int, default=10, help='scans a moyenner')
    args = ap.parse_args()
    rclpy.init()
    node = Zero(math.radians(args.azimut), args.tours)
    print(f'  posez l objet a {args.azimut:+.0f} deg, entre 30 cm et 1.50 m, le reste degage...')
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
