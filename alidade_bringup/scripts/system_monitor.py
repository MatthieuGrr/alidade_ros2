#!/usr/bin/env python3
"""Supervision du Pi et des cadences capteurs, en diagnostics standard.

Publie sur /diagnostics, a cadence fixe : temperature et throttling, rails du
PMIC, CPU, memoire, disque, WiFi, synchro NTP, et la cadence de chaque topic
surveille (LiDAR, camera, IMU, odometrie) face a celle que la revision
materielle promet. Le diagnostic_aggregator les regroupe ensuite avec ceux du
Pico (alidade_hardware) et de nav2 dans /diagnostics_agg.

Hors du Pi (simulation, conteneur), les sources absentes sont dites absentes,
pas inventees : pas de vcgencmd, pas de WiFi, tout le reste fonctionne.
"""
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

from alidade_bringup import telemetry

THRESHOLDS = ('temp_warn_c', 'temp_error_c', 'rail_5v_warn_v', 'cpu_warn_percent',
              'mem_warn_percent', 'disk_warn_percent', 'wifi_warn_dbm', 'rate_warn_ratio')
DEFAULTS = (75.0, 85.0, 4.80, 85.0, 85.0, 85.0, -75.0, 0.8)


class RateWatch:
    """Compte les messages d'un topic ; la cadence est moyennee sur une fenetre."""

    def __init__(self, node: Node, topic: str, type_name: str, expected: float,
                 optional: bool = False, window_s: float = 3.0):
        self.topic = topic
        self.expected = expected
        self.optional = optional
        self.window_s = window_s
        self.stamps = []
        self.node = node
        self.sub = node.create_subscription(get_message(type_name), topic, self.on_msg,
                                            qos_profile_sensor_data)

    def on_msg(self, _msg):
        self.stamps.append(time.monotonic())

    def hz(self) -> float | None:
        now = time.monotonic()
        self.stamps = [t for t in self.stamps if now - t <= self.window_s]
        if not self.stamps:
            return None
        return len(self.stamps) / self.window_s


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.declare_parameter('period', 1.0)
        for name, default in zip(THRESHOLDS, DEFAULTS):
            self.declare_parameter(name, default)
        # "topic|type|hz[|optional]" : une chaine par entree, pour ne pas
        # melanger les types dans une liste de parametres.
        self.declare_parameter('watch', [''])

        self.thresholds = {name: float(self.get_parameter(name).value) for name in THRESHOLDS}
        self.watches = []
        for spec in self.get_parameter('watch').value:
            if not spec:
                continue
            fields = [p.strip() for p in spec.split('|')]
            topic, type_name, hz = fields[:3]
            optional = len(fields) > 3 and fields[3] == 'optional'
            self.watches.append(RateWatch(self, topic, type_name, float(hz), optional))

        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.started = time.monotonic()
        telemetry.read_system()  # amorce cpu_percent, dont la premiere lecture vaut toujours 0
        period = float(self.get_parameter('period').value)
        self.create_timer(period, self.on_timer)
        self.get_logger().info(
            f'supervision a {1 / period:.1f} Hz, {len(self.watches)} topics surveilles : '
            + ', '.join(w.topic for w in self.watches))

    def on_timer(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = telemetry.statuses(telemetry.sample(), self.thresholds)
        # Dix secondes de grace au demarrage : le LiDAR met du temps a venir,
        # ce n'est pas une erreur tant que la pile se met en place.
        grace = time.monotonic() - self.started < 10.0
        for w in self.watches:
            st = telemetry.rate_status(w.topic, w.hz(), w.expected, self.thresholds['rate_warn_ratio'],
                                       optional=w.optional)
            if grace and st.level == telemetry.ERROR:
                st.level = telemetry.WARN
                st.message = 'en attente (demarrage)'
            array.status.append(st)
        self.pub.publish(array)


def main():
    rclpy.init()
    node = SystemMonitor()
    try:
        rclpy.spin(node)
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
