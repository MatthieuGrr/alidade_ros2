#!/usr/bin/env python3
"""Compare l'odometrie de diff_drive_controller a la verite terrain Gazebo.

    ros2 run alidade_sim measure_odom.py            # simulation lancee, robot dans un espace degage

Ligne droite de 0.2 m/s pendant 6 s puis rotation de 1 rad/s pendant 3 s ;
la verite terrain est lue avec `gz model -m alidade -p`. Le robot doit avoir
plus de 1.5 m devant lui : un mur fait patiner les roues et fausse tout.
"""
import math, subprocess, sys, time, re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


def truth():
    out = subprocess.run(['gz', 'model', '-m', 'alidade', '-p'], capture_output=True, text=True, timeout=10).stdout
    m = re.search(r'Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\[([^\]]+)\]\s*\[([^\]]+)\]', out)
    xyz = [float(v) for v in m.group(1).split()]
    rpy = [float(v) for v in m.group(2).split()]
    return xyz[0], xyz[1], rpy[2]


class Meter(Node):
    def __init__(self):
        super().__init__('measure_odom')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.odom = None
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.cb, 10)

    def cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def drive(self, vx, wz, seconds):
        end = time.time() + seconds
        while time.time() < end:
            m = TwistStamped(); m.header.stamp = self.get_clock().now().to_msg()
            m.twist.linear.x = vx; m.twist.angular.z = wz
            self.pub.publish(m); rclpy.spin_once(self, timeout_sec=0.05)
        for _ in range(30):
            m = TwistStamped(); m.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(m); rclpy.spin_once(self, timeout_sec=0.05)

    def wait_odom(self):
        while self.odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def settle(n, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--linear-seconds', type=float, default=6.0)
    ap.add_argument('--angular-seconds', type=float, default=3.0)
    ap.add_argument('--settle', type=float, default=2.0, help='attente a l arret avant lecture (s)')
    args = ap.parse_args()
    rclpy.init()
    n = Meter(); n.wait_odom()
    settle(n, args.settle)
    o0, t0 = n.odom, truth()
    n.drive(0.2, 0.0, args.linear_seconds); settle(n, args.settle)
    o1, t1 = n.odom, truth()
    d_odom = math.dist(o0[:2], o1[:2]); d_true = math.dist(t0[:2], t1[:2])
    print(f'ligne droite : odom {d_odom:.4f} m, verite {d_true:.4f} m, erreur {100*(d_odom-d_true)/max(d_true, 1e-9):+.2f} %')
    o0, t0 = o1, t1
    n.drive(0.0, 1.0, args.angular_seconds); settle(n, args.settle)
    o1, t1 = n.odom, truth()
    y_odom = wrap(o1[2] - o0[2]); y_true = wrap(t1[2] - t0[2])
    print(f'rotation     : odom {math.degrees(y_odom):.2f} deg, verite {math.degrees(y_true):.2f} deg, erreur {100*(y_odom-y_true)/max(abs(y_true), 1e-9):+.2f} %')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
