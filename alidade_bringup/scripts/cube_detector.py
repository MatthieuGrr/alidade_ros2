#!/usr/bin/env python3
"""Pose du cube cible depuis les detections ArUco, pose d'arret du docking.

    /aruco_detections  ->  /cube/pose       (PoseStamped, repere de la camera)
                           /cube/dock_pose  (PoseStamped, base_footprint : ou
                                             s'arreter devant le cube, pour
                                             cube_docking)
                           /cube/center     (PointStamped, base_footprint :
                                             centre du cube, publie juste avant
                                             la pose d'arret)
                           /cube/markers    (MarkerArray, pour Vizanti ou rviz)

Chaque face vue donne la pose du cube ; plusieurs faces sont moyennees (voir
alidade_bringup/cube.py). Rien n'est publie quand le cube n'est pas vu : les
marqueurs ont une duree de vie courte et s'effacent d'eux-memes, et le
docking s'arrete faute de pose fraiche.
"""
import math

import numpy as np
import rclpy
import tf2_ros
from aruco_opencv_msgs.msg import ArucoDetection
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from alidade_bringup import cube, params

ORANGE = ColorRGBA(r=1.0, g=0.55, b=0.1, a=0.9)
RED = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.9)
DISAGREE_DEG = 15.0   # au-dela, des faces se contredisent : cube en rouge
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
GREEN = ColorRGBA(r=0.2, g=0.85, b=0.3, a=0.9)
BASE = 'base_footprint'


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')
        path = self.declare_parameter('cube_file', str(params.bringup_share() / 'config' / 'cube.yaml')).value
        self.lifetime = self.declare_parameter('marker_lifetime', 0.5).value
        gap = self.declare_parameter('gap', 0.10).value
        front = self.declare_parameter('front_x', 0.120).value
        self.hysteresis = self.declare_parameter('hysteresis_deg', 15.0).value
        self.standoff = gap + front
        self.edge, self.faces = cube.load(path)
        self.side = None            # face visee, gardee d'une image a l'autre
        self.last_seen = None
        # La camera est fixe sur le robot : sa pose dans base_footprint ne se
        # lit qu'une fois, puis on cesse d'ecouter /tf (des centaines de
        # messages par seconde a desserialiser en Python pour rien).
        self.camera_in_base = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub_pose = self.create_publisher(PoseStamped, '/cube/pose', 10)
        self.pub_dock = self.create_publisher(PoseStamped, '/cube/dock_pose', 10)
        self.pub_centre = self.create_publisher(PointStamped, '/cube/center', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/cube/markers', 10)
        self.create_subscription(ArucoDetection, '/aruco_detections', self.on_detection, 10)
        self.get_logger().info(f'cube de {self.edge * 1000:.0f} mm, faces {sorted(self.faces)}')

    def camera_pose(self, frame):
        """Pose 4x4 du repere de la camera dans base_footprint, lue une fois."""
        if self.camera_in_base is None:
            try:
                t = self.tf_buffer.lookup_transform(BASE, frame, rclpy.time.Time()).transform
            except tf2_ros.TransformException:
                return None
            self.camera_in_base = cube.matrix(
                [t.translation.x, t.translation.y, t.translation.z],
                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
            self.tf_listener.unregister()
            self.get_logger().info(f'camera {frame} lue dans {BASE}, plus d ecoute de /tf')
        return self.camera_in_base

    def on_detection(self, msg):
        markers = {}
        for m in msg.markers:
            p, q = m.pose.position, m.pose.orientation
            markers[m.marker_id] = cube.matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        est = cube.estimate(markers, self.faces)
        if est is None:
            return
        q = cube.quaternion(est.rotation)
        pose = Pose(position=Point(x=est.position[0], y=est.position[1], z=est.position[2]),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
        self.pub_pose.publish(PoseStamped(header=msg.header, pose=pose))
        target = self.dock_target(msg, est)
        self.pub_markers.publish(MarkerArray(markers=self.display(msg.header, pose, est, target)))

    def dock_target(self, msg, est):
        now = self.get_clock().now()
        if self.last_seen is not None and (now - self.last_seen).nanoseconds > 1e9:
            self.side = None        # cube perdu : la prochaine face se choisit librement
        self.last_seen = now
        cam = self.camera_pose(msg.header.frame_id)
        if cam is None:
            return None
        position = cam[:3, :3] @ est.position + cam[:3, 3]
        target = cube.dock_target(position, cam[:3, :3] @ est.rotation, self.edge, self.standoff,
                                  self.side, self.hysteresis)
        if target is None:
            return None
        self.side = target.side
        dock = PoseStamped()
        dock.header.stamp = msg.header.stamp      # instant de l'image, pas de la publication
        dock.header.frame_id = BASE
        self.pub_centre.publish(PointStamped(header=dock.header, point=Point(
            x=float(position[0]), y=float(position[1]), z=float(position[2]))))
        dock.pose.position = Point(x=target.x, y=target.y, z=0.0)
        dock.pose.orientation = Quaternion(z=math.sin(target.yaw / 2), w=math.cos(target.yaw / 2))
        self.pub_dock.publish(dock)
        return dock, target

    def display(self, header, pose, est, target):
        life = Duration(sec=int(self.lifetime), nanosec=int(self.lifetime % 1 * 1e9))

        def marker(mid, kind, **kw):
            return Marker(header=header, ns='cube', id=mid, type=kind, action=Marker.ADD,
                          lifetime=life, **kw)

        body = marker(0, Marker.CUBE, pose=pose, color=RED if est.angle > DISAGREE_DEG else ORANGE)
        body.scale.x = body.scale.y = body.scale.z = self.edge
        # Fleche vers la face « devant » (x du cube), a partir de son centre.
        front = marker(1, Marker.ARROW, pose=pose, color=WHITE)
        front.scale.x, front.scale.y, front.scale.z = self.edge, 0.006, 0.006
        label = marker(2, Marker.TEXT_VIEW_FACING, color=WHITE,
                       text=f"cube {', '.join(map(str, est.faces))}"
                            + (f'  -> face {target[1].face_id}' if target else '')
                            + (f'  ecart {est.spread * 1000:.0f} mm / {est.angle:.0f} deg'
                               if len(est.faces) > 1 else ''))
        # Au-dessus du cube, le long de son axe « haut » : la verticale tant
        # qu'il est pose sur sa face 6.
        up = est.rotation @ np.array([0.0, 0.0, 1.0])
        above = est.position + up * self.edge * 1.2
        label.pose.position = Point(x=above[0], y=above[1], z=above[2])
        label.pose.orientation.w = 1.0
        label.scale.z = 0.025
        shown = [body, front, label]
        if target:
            # Ou base_footprint s'arretera, et son cap : ce que suit le docking.
            dock, _ = target
            stop = Marker(header=dock.header, ns='dock', id=0, type=Marker.ARROW, action=Marker.ADD,
                          lifetime=life, pose=dock.pose, color=GREEN)
            stop.scale.x, stop.scale.y, stop.scale.z = 0.08, 0.012, 0.012
            shown.append(stop)
        return shown


def main():
    rclpy.init()
    rclpy.spin(CubeDetector())


if __name__ == '__main__':
    main()
