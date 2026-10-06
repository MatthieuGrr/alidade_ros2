#!/usr/bin/env python3
"""Barriere de launch : se termine au premier message recu sur un topic
LaserScan, ou apres un delai maximal.

nav2 n'a aucun sens sans scan : sa costmap globale attend le repere map, que
slam_toolbox ne publie qu'apres avoir traite un premier scan, et abandonne
apres initial_transform_timeout (60 s) : sans barriere le gestionnaire de
cycle de vie de nav2 abandonne tout le bringup et ne reessaie jamais. On
enchaine donc nav.launch.py sur la fin de ce processus.

Cette barriere a d'abord servi a encaisser les 15 s a 7 minutes que mettait
l'ancien pilote apt a demarrer le C1. sllidar_ros2 demarre en une demi-seconde
et la barriere n'a plus ce role, mais elle garde le sien : un LiDAR debranche,
un cable coupe ou un capteur en panne ne doivent pas faire tomber nav2 en
silence.

    ros2 run alidade_bringup wait_for_scan.py --ros-args -p topic:=/scan -p timeout:=600.0
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class WaitForScan(Node):
    def __init__(self):
        super().__init__('wait_for_scan')
        self.declare_parameter('topic', '/scan')
        self.declare_parameter('timeout', 600.0)
        self.topic = self.get_parameter('topic').value
        self.timeout = float(self.get_parameter('timeout').value)
        self.received = False
        self.create_subscription(LaserScan, self.topic, self.on_scan, qos_profile_sensor_data)
        self.started = self.get_clock().now()
        self.create_timer(5.0, self.on_tick)
        self.get_logger().info(f'attente du premier message sur {self.topic} ({self.timeout:.0f} s au plus)')

    def on_scan(self, msg):
        if not self.received:
            self.received = True
            waited = (self.get_clock().now() - self.started).nanoseconds * 1e-9
            self.get_logger().info(f'{self.topic} recu apres {waited:.1f} s ({len(msg.ranges)} points)')

    def on_tick(self):
        waited = (self.get_clock().now() - self.started).nanoseconds * 1e-9
        if waited >= self.timeout:
            self.get_logger().warn(f'toujours rien sur {self.topic} apres {waited:.0f} s, on continue quand meme')
            self.received = True


def main():
    rclpy.init()
    node = WaitForScan()
    try:
        while rclpy.ok() and not node.received:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
