#!/usr/bin/env python3
"""Campagne de docking sur le vrai robot : essais enchaines, ecart final REEL.

    dock_campaign.py n:0.15,n:0.15,f:0.15

Chaque essai est decrit par 'type:latence' : n = depart d'ou se trouve le
robot, f = recul de 40 cm d'abord ; la latence est posee a chaud sur
cube_docking (detection_latency). Les departs sont perturbes par une rotation
(0, +10, -10, +6, -6 deg). Robot en mode DOCKING possible, cube pose au sol.

L'ecart final se mesure mal de tres pres : la camera n'y voit plus que le
dessus du cube, flou, au bord de l'image. Apres chaque arret, le robot recule
donc d'une distance connue a l'odometrie (juste a 0.5 % pres, verifie par
ICP) jusqu'a ~30 cm, ou la camera est fiable, et l'ecart reel a l'arret vaut
l'ecart mesure la-bas moins le recul. Degager un metre autour du robot.
"""
import math
import re
import statistics
import subprocess
import sys
import time

import rclpy
from alidade_msgs.msg import RobotState
from alidade_msgs.srv import SetMode
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import PointStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger

if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help'):
    print(__doc__)
    sys.exit(0)
# Essais : 'n:0.15' = depart d'ou est le robot ; 'f:0.15' = recul de 40 cm d'abord.
SPEC = [x.split(':') for x in (sys.argv[1] if len(sys.argv) > 1 else 'n:0.35').split(',')]
TRIALS = len(SPEC)
PERTURB = [0.0, 10.0, -10.0, 6.0, -6.0, 0.0]      # rotation avant chaque essai, deg
FRONT, HALF, GAP = 0.120, 0.025, 0.050


class Campaign(Node):
    def __init__(self):
        super().__init__('dock_campaign')
        self.state = None
        self.pos = None
        self.yaw = None
        self.centres = []
        self.ids = []
        self.create_subscription(RobotState, '/robot/state', lambda m: setattr(self, 'state', m), 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(PointStamped, '/cube/center', lambda m: self.centres.append((time.time(), m.point.x, m.point.y)), 20)
        self.create_subscription(ArucoDetection, '/aruco_detections',
                                 lambda m: self.ids.append(sorted(k.marker_id for k in m.markers)), 20)
        self.cmd = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.set_mode = self.create_client(SetMode, '/mode_manager/set_mode')
        self.undock = self.create_client(Trigger, '/mode_manager/undock')

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pos = (p.x, p.y)
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def spin(self, d):
        end = time.time() + d
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, client, req, timeout=10.0):
        client.wait_for_service(timeout_sec=5.0)
        fut = client.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result()

    def stop(self):
        for _ in range(5):
            self.cmd.publish(Twist())
            self.spin(0.03)
        self.spin(0.6)

    def turn(self, deg):
        start, goal, t0 = self.yaw, math.radians(deg), time.time()
        while time.time() - t0 < 12:
            self.spin(0.03)
            err = goal - math.atan2(math.sin(self.yaw - start), math.cos(self.yaw - start))
            if abs(err) < math.radians(2.0):
                break
            self.cmd.publish(Twist(angular=Vector3(z=math.copysign(min(max(abs(1.5 * err), 0.5), 0.8), err))))
        self.stop()

    def drive(self, dist):
        p0, t0 = self.pos, time.time()
        while time.time() - t0 < 15:
            self.spin(0.03)
            if math.dist(self.pos, p0) >= abs(dist) - 0.003:
                break
            self.cmd.publish(Twist(linear=Vector3(x=math.copysign(0.08, dist))))
        self.stop()
        return math.dist(self.pos, p0)

    def camera_gap(self, seconds=3.0):
        """Ecart face avant -> face du cube vu par la camera, et faces vues."""
        self.centres.clear(); self.ids.clear()
        self.spin(seconds)
        faces = sorted({i for f in self.ids for i in f})
        if not self.centres:
            return None, faces
        cx = statistics.median(c[1] for c in self.centres)
        cy = statistics.median(c[2] for c in self.centres)
        return math.hypot(cx, cy) - HALF - FRONT, faces

    def wait_result(self, timeout=60.0):
        end = time.time() + timeout
        self.spin(2.0)
        while time.time() < end:
            self.spin(0.5)
            sub = self.state.substate if self.state else ''
            if 'arrete devant' in sub or 'echou' in sub or 'impossible' in sub or 'annul' in sub:
                return sub
        return 'delai depasse'


def journal_since(t0):
    out = subprocess.run(['journalctl', '-u', 'alidade', '--since', f'@{int(t0)}', '--no-pager'],
                         capture_output=True, text=True).stdout
    arrive = re.findall(r'arrive : ([+-]\d+) mm sur l axe, ([+-]\d+) mm lateral, ([+-][\d.]+) deg, ([\d.]+) s, (\d+) reprise', out)
    blind = re.findall(r'cube hors champ a ([\d.]+) cm', out)
    warn = re.findall(r'cube_docking\]: (.*(?:reprise|echec|perdu).*)', out)
    return (arrive[-1] if arrive else None), (blind[-1] if blind else None), warn


rclpy.init()
c = Campaign()
while c.state is None or c.pos is None:
    rclpy.spin_once(c, timeout_sec=0.1)
rows = []
for k in range(TRIALS):
    kind, latency = SPEC[k][0], float(SPEC[k][1])
    perturb = PERTURB[k % len(PERTURB)]
    subprocess.run(['ros2', 'param', 'set', '/cube_docking', 'detection_latency', str(latency)],
                   capture_output=True, text=True)
    print(f"\n=== essai {k + 1} : {'loin' if kind == 'f' else 'pres'}, latence {latency} s, rotation {perturb:+.0f} deg")
    if kind == 'f':
        c.drive(-0.40)
    if perturb:
        c.turn(perturb)
    start_gap, start_faces = c.camera_gap(2.0)
    print(f"   depart : ecart camera {start_gap*100 if start_gap else float('nan'):.1f} cm, faces {start_faces}")
    t0 = time.time()
    res = c.call(c.set_mode, SetMode.Request(mode=5))
    print(f"   DOCKING : {res.message if res else 'pas de reponse'}")
    if not res or not res.ok:
        rows.append((k + 1, f"{kind}{latency}", 'refuse', None, None, None, None))
        continue
    outcome = c.wait_result()
    t_total = time.time() - t0
    arrive, blind, warn = journal_since(t0)
    near_gap, near_faces = c.camera_gap(2.0)
    # Recul connu, puis mesure de l'ecart la ou la camera est fiable.
    c.call(c.undock, Trigger.Request())
    p_before = c.pos
    c.spin(8.0)
    back = math.dist(c.pos, p_before)
    back += c.drive(-0.08)
    far_gap, far_faces = c.camera_gap(3.0)
    true_gap = far_gap - back if far_gap is not None else None
    print(f"   {outcome} en {t_total:.1f} s ; journal : {arrive} ; aveugle : {blind} ; alertes : {warn}")
    print(f"   a l'arret, camera : {near_gap*100 if near_gap else float('nan'):.1f} cm (faces {near_faces})")
    print(f"   recul {back*100:.1f} cm puis camera {far_gap*100 if far_gap else float('nan'):.1f} cm (faces {far_faces})"
          f"  ->  ECART REEL A L'ARRET {true_gap*100 if true_gap is not None else float('nan'):.1f} cm")
    rows.append((k + 1, f"{kind}{latency}", outcome, arrive, blind, near_gap, true_gap))

print("\n=== bilan")
print(" essai | cas    | resultat               | axe   lat   cap    duree reprises | aveugle | camera a l'arret | ecart reel")
for k, perturb, outcome, arrive, blind, near, true in rows:
    a = f"{arrive[0]:>4} {arrive[1]:>4} {arrive[2]:>5} {arrive[3]:>5}s {arrive[4]:>3}" if arrive else " " * 29
    print(f"  {k:3d}  | {perturb:6s} | {outcome[:22]:22s} | {a} | {blind or '-':>7} | "
          f"{(near*100 if near else float('nan')):6.1f} cm        | {(true*100 if true is not None else float('nan')):5.1f} cm")
rclpy.shutdown()
