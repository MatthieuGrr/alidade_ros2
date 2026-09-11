#!/usr/bin/env python3
"""Pose du cube cible depuis les detections ArUco, et son affichage.

    /aruco_detections  ->  /cube/pose     (PoseStamped, repere de la camera)
                           /cube/markers  (MarkerArray, pour Vizanti ou rviz)

Chaque face vue donne la pose du cube ; plusieurs faces sont moyennees (voir
alidade_bringup/cube.py). Rien n'est publie quand le cube n'est pas vu : les
marqueurs ont une duree de vie courte et s'effacent d'eux-memes.
"""
import numpy as np
import rclpy
from aruco_opencv_msgs.msg import ArucoDetection
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from alidade_bringup import cube, params

ORANGE = ColorRGBA(r=1.0, g=0.55, b=0.1, a=0.9)
RED = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.9)
DISAGREE_DEG = 15.0   # au-dela, des faces se contredisent : cube en rouge
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')
        path = self.declare_parameter('cube_file', str(params.bringup_share() / 'config' / 'cube.yaml')).value
        self.lifetime = self.declare_parameter('marker_lifetime', 0.5).value
        self.edge, self.faces = cube.load(path)
        self.pub_pose = self.create_publisher(PoseStamped, '/cube/pose', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/cube/markers', 10)
        self.create_subscription(ArucoDetection, '/aruco_detections', self.on_detection, 10)
        self.get_logger().info(f'cube de {self.edge * 1000:.0f} mm, faces {sorted(self.faces)}')

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
        self.pub_markers.publish(MarkerArray(markers=self.display(msg.header, pose, est)))

    def display(self, header, pose, est):
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
                            + (f'  ecart {est.spread * 1000:.0f} mm / {est.angle:.0f} deg'
                               if len(est.faces) > 1 else ''))
        # Au-dessus du cube, le long de son axe « haut » : la verticale tant
        # qu'il est pose sur sa face 6.
        up = est.rotation @ np.array([0.0, 0.0, 1.0])
        above = est.position + up * self.edge * 1.2
        label.pose.position = Point(x=above[0], y=above[1], z=above[2])
        label.pose.orientation.w = 1.0
        label.scale.z = 0.025
        return [body, front, label]


def main():
    rclpy.init()
    rclpy.spin(CubeDetector())


if __name__ == '__main__':
    main()
