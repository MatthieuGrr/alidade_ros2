#!/usr/bin/env python3
"""Pierre a aborder, choisie parmi les marqueurs vus, et pose d'arret du docking.

    /aruco_detections  ->  /stone/pose       (PoseStamped, base_footprint : le
                                              marqueur vise, redresse)
                           /stone/dock_pose  (PoseStamped : ou base_footprint
                                              doit s'arreter, pour docking_server)
                           /stone/center     (PointStamped : centre du marqueur
                                              vise, publie juste avant la pose
                                              d'arret -- c'est le bras de levier
                                              du lissage)
                           /stone/markers    (MarkerArray, pour Vizanti ou rviz)

Toutes les pierres portent le MEME identifiant : le tri est geometrique et vit
dans alidade_bringup/stone.py. Ici, la plomberie ROS et l'affichage -- chaque
marqueur rejete est montre en rouge avec le MOTIF de son rejet, sans quoi un
reglage de seuil se debogue a l'aveugle.

Rien n'est publie quand aucune pierre n'est retenue : les markers ont une duree
de vie courte et s'effacent d'eux-memes, et le docking s'arrete faute de pose
fraiche.
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

from alidade_bringup import params, stone

GREEN = ColorRGBA(r=0.2, g=0.85, b=0.3, a=0.9)      # la cible
ORANGE = ColorRGBA(r=1.0, g=0.55, b=0.1, a=0.9)     # candidat non retenu
RED = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.9)         # rejete
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
BASE = 'base_footprint'


class StoneDetector(Node):
    def __init__(self):
        super().__init__('stone_detector')
        p = lambda name, default: self.declare_parameter(name, default).value
        path = p('stone_file', str(params.bringup_share() / 'config' / 'stone.yaml'))
        self.lifetime = p('marker_lifetime', 0.5)
        self.standoff = p('gap', 0.10) + p('front_x', 0.120)
        self.limits = stone.Limits(
            tilt_max=p('tilt_max_deg', 25.0),
            height_min=p('height_min', 0.030), height_max=p('height_max', 0.090),
            range_min=p('range_min', 0.10), range_max=p('range_max', 0.90),
            incidence_max=p('incidence_max_deg', 70.0),
            stack_radius=p('stack_radius', 0.080), level_tol=p('level_tolerance', 0.030),
            lock_radius=p('lock_radius', 0.100), tie=p('tie', 'proche'))
        self.lock_timeout = p('lock_timeout', 1.0)
        self.spec = stone.load(path)
        self.locked = None          # centre de la cible verrouillee, repere du robot
        self.last_seen = None
        # La camera est fixe sur le robot : sa pose dans base_footprint ne se lit
        # qu'une fois, puis on cesse d'ecouter /tf (des centaines de messages par
        # seconde a desserialiser en Python pour rien).
        self.camera_in_base = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub_pose = self.create_publisher(PoseStamped, '/stone/pose', 10)
        self.pub_dock = self.create_publisher(PoseStamped, '/stone/dock_pose', 10)
        self.pub_centre = self.create_publisher(PointStamped, '/stone/center', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/stone/markers', 10)
        self.create_subscription(ArucoDetection, '/aruco_detections', self.on_detection, 10)
        self.get_logger().info(
            f'{self.spec.name} {"x".join(f"{v*1000:.0f}" for v in self.spec.size)} mm, '
            f'marqueur {self.spec.marker_dict} id {self.spec.marker_id} de '
            f'{self.spec.marker_size * 1000:.0f} mm, arret a '
            f'{(self.standoff - 0.120) * 100:.0f} cm de la face')

    def camera_pose(self, frame):
        """Pose 4x4 du repere de la camera dans base_footprint, lue une fois."""
        if self.camera_in_base is None:
            try:
                t = self.tf_buffer.lookup_transform(BASE, frame, rclpy.time.Time()).transform
            except tf2_ros.TransformException:
                return None
            self.camera_in_base = stone.matrix(
                [t.translation.x, t.translation.y, t.translation.z],
                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
            self.tf_listener.unregister()
            self.get_logger().info(f'camera {frame} lue dans {BASE}, plus d ecoute de /tf')
        return self.camera_in_base

    def on_detection(self, msg):
        cam = self.camera_pose(msg.header.frame_id)
        if cam is None:
            return
        # Tout se juge dans le repere du robot, ou le plateau est connu : c'est
        # la hauteur du marqueur au-dessus du sol qui dit ce qu'on regarde.
        detections = []
        for m in msg.markers:
            p, q = m.pose.position, m.pose.orientation
            detections.append((m.marker_id,
                               cam @ stone.matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])))
        tags = stone.assess(detections, self.spec, cam[:3, 3], self.limits)
        target = stone.select(tags, self.limits, self.release_lock())
        header = msg.header.__class__(stamp=msg.header.stamp, frame_id=BASE)
        dock = self.publish_target(header, target) if target is not None else None
        self.pub_markers.publish(MarkerArray(markers=self.display(header, tags, target, dock)))

    def release_lock(self):
        """Verrou de la cible, lache apres `lock_timeout` sans aucune pierre vue."""
        now = self.get_clock().now()
        if self.last_seen is not None and \
                (now - self.last_seen).nanoseconds > self.lock_timeout * 1e9:
            self.locked = None
        return self.locked

    def publish_target(self, header, target):
        self.locked = target.centre
        self.last_seen = self.get_clock().now()
        dock = stone.dock_target(target, self.standoff)
        # Instant de l'IMAGE, pas de la publication : docking_server s'en sert
        # pour retrouver ou etait le robot quand la pierre a ete vue.
        self.pub_pose.publish(PoseStamped(header=header, pose=self.pose_of(
            target.centre, target.yaw)))
        self.pub_centre.publish(PointStamped(header=header, point=Point(
            x=float(target.centre[0]), y=float(target.centre[1]), z=float(target.centre[2]))))
        stop = PoseStamped(header=header, pose=self.pose_of([dock.x, dock.y, 0.0], dock.yaw))
        self.pub_dock.publish(stop)
        return dock

    @staticmethod
    def pose_of(position, yaw):
        return Pose(position=Point(x=float(position[0]), y=float(position[1]), z=float(position[2])),
                    orientation=Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2)))

    def display(self, header, tags, target, dock):
        life = Duration(sec=int(self.lifetime), nanosec=int(self.lifetime % 1 * 1e9))

        def marker(ns, mid, kind, **kw):
            return Marker(header=header, ns=ns, id=mid, type=kind, action=Marker.ADD,
                          lifetime=life, **kw)

        shown = []
        for i, tag in enumerate(tags):
            chosen = target is not None and tag is target
            colour = GREEN if chosen else (ORANGE if tag.ok else RED)
            # Le marqueur lui-meme, a plat sur la face : une dalle mince, orientee
            # comme la normale redressee.
            plate = marker('tags', i, Marker.CUBE, color=colour,
                           pose=self.pose_of(tag.centre, tag.yaw))
            plate.scale.x, plate.scale.y = 0.004, self.spec.marker_size
            plate.scale.z = self.spec.marker_size
            label = marker('labels', i, Marker.TEXT_VIEW_FACING, color=colour,
                           text=f'{tag.height * 1000:.0f} mm'
                                + (f'  {tag.reason}' if tag.reason else
                                   ('  CIBLE' if chosen else '  candidat')))
            label.pose = self.pose_of(tag.centre + np.array([0.0, 0.0, 0.06]), 0.0)
            label.scale.z = 0.025
            shown += [plate, label]
        if target is not None:
            # La pierre deduite du marqueur vise : son centre est a une
            # demi-largeur derriere la face, son grand axe perpendiculaire.
            normal = np.array([math.cos(target.yaw), math.sin(target.yaw), 0.0])
            centre = target.centre - normal * (self.spec.size[1] / 2)
            centre[2] = self.spec.size[2] / 2
            body = marker('stone', 0, Marker.CUBE, color=ORANGE,
                          pose=self.pose_of(centre, stone.wrap(target.yaw + math.pi / 2)))
            body.scale.x, body.scale.y, body.scale.z = self.spec.size
            body.color = ColorRGBA(r=0.75, g=0.6, b=0.4, a=0.45)      # carton
            # Ou base_footprint s'arretera, et son cap : ce que suit le docking.
            stop = marker('dock', 0, Marker.ARROW, color=GREEN,
                          pose=self.pose_of([dock.x, dock.y, 0.0], dock.yaw))
            stop.scale.x, stop.scale.y, stop.scale.z = 0.08, 0.012, 0.012
            shown += [body, stop]
        return shown


def main():
    rclpy.init()
    rclpy.spin(StoneDetector())


if __name__ == '__main__':
    main()
