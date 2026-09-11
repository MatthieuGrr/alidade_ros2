#!/usr/bin/env python3
"""Docking sur le cube : actions /dock_robot et /undock_robot (nav2_msgs).

    /cube/dock_pose (base_footprint, instant de l'image) --+
    /diff_drive_controller/odom ---------------------------+--> /cmd_vel_nav

Les types d'action sont ceux du serveur de docking de nav2, pour qu'un
client (le gestionnaire de modes) n'ait pas a savoir qui les sert ; la
commande est la notre (alidade_bringup/docking.py explique pourquoi). Les
vitesses passent par le lisseur et le moniteur de collision, comme toutes
les autres.

Chaque pose d'arret detectee est ramenee dans odom avec la pose du robot a
l'instant de l'IMAGE, prise dans un historique de l'odometrie : sans cela,
les 100 a 200 ms de latence de la detection deplaceraient la cible d'autant
de chemin parcouru. Le cube etant immobile, la cible est ensuite lissee dans
odom.
"""
import math
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav2_msgs.action import DockRobot, UndockRobot
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from alidade_bringup import docking
from alidade_bringup.docking import Pose2D

FB = DockRobot.Feedback
RES = DockRobot.Result


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """b, exprime dans le repere a, ramene dans le repere parent de a."""
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, docking.wrap(a.yaw + b.yaw))


class Canceled(Exception):
    pass


class Failure(Exception):
    def __init__(self, code: int, text: str):
        super().__init__(text)
        self.code = code


class CubeDocking(Node):
    def __init__(self):
        super().__init__('cube_docking')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.g = docking.Gains(
            staging=p('staging', 0.20), v_max=p('v_max', 0.08), v_min=p('v_min', 0.02),
            w_max=p('w_max', 0.8), w_min=p('w_min', 0.25), k_v=p('k_v', 1.0),
            k_yaw=p('k_yaw', 2.0), k_lat=p('k_lat', 10.0),
            lat_tol=p('lateral_tolerance', 0.015),
            arrive_yaw_tol=math.radians(p('yaw_tolerance_deg', 5.0)))
        self.rate = p('rate', 20.0)
        self.detection_timeout = p('detection_timeout', 1.0)
        self.filter_coef = p('filter_coef', 0.3)
        self.max_retries = p('max_retries', 2)
        self.undock_distance = p('undock_distance', 0.15)
        self.stage_timeout = p('stage_timeout', 30.0)

        self.lock = threading.Lock()
        self.odom = deque(maxlen=150)          # (ns, Pose2D), 3 s a 50 Hz
        self.target = None                     # pose d'arret lissee, dans odom
        self.seen_at = None                    # derniere detection, horloge ROS
        # Delais sur l'horloge ROS, pas sur l'horloge murale : en simulation,
        # le temps simule peut aller cinq fois moins vite.
        self.busy = threading.Lock()

        group = ReentrantCallbackGroup()
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.on_odom, 20, callback_group=group)
        self.create_subscription(PoseStamped, '/cube/dock_pose', self.on_dock_pose, 10, callback_group=group)
        self.cmd = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        for action, name, run in ((DockRobot, 'dock_robot', self.run_dock),
                                  (UndockRobot, 'undock_robot', self.run_undock)):
            ActionServer(self, action, name, execute_callback=run, goal_callback=self.on_goal,
                         cancel_callback=lambda _goal: CancelResponse.ACCEPT, callback_group=group)

    # ------------------------------------------------------------------ entrees
    def on_odom(self, msg):
        pose = msg.pose.pose
        stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
        with self.lock:
            self.odom.append((stamp, Pose2D(pose.position.x, pose.position.y, yaw_of(pose.orientation))))

    def robot_at(self, stamp_ns: int | None = None) -> Pose2D | None:
        """Pose du robot dans odom, a un instant de l'historique ou la derniere."""
        with self.lock:
            if not self.odom:
                return None
            if stamp_ns is None:
                return self.odom[-1][1]
            return min(self.odom, key=lambda e: abs(e[0] - stamp_ns))[1]

    def on_dock_pose(self, msg):
        robot = self.robot_at(rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds)
        if robot is None:
            return
        p = msg.pose
        seen = compose(robot, Pose2D(p.position.x, p.position.y, yaw_of(p.orientation)))
        now = self.now()
        with self.lock:
            t, k = self.target, self.filter_coef
            if t is None or self.seen_at is None or now - self.seen_at > 2 * self.detection_timeout:
                self.target = seen
            else:
                self.target = Pose2D(t.x + k * (seen.x - t.x), t.y + k * (seen.y - t.y),
                                     docking.wrap(t.yaw + k * docking.wrap(seen.yaw - t.yaw)))
            self.seen_at = now

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def fresh_target(self) -> Pose2D | None:
        now = self.now()
        with self.lock:
            if self.seen_at is None or now - self.seen_at > self.detection_timeout:
                return None
            return self.target

    # ------------------------------------------------------------------ actions
    def on_goal(self, _goal):
        return GoalResponse.REJECT if self.busy.locked() else GoalResponse.ACCEPT

    def drive(self, v: float, w: float):
        self.cmd.publish(Twist(linear=Vector3(x=v), angular=Vector3(z=w)))

    def stop(self):
        self.drive(0.0, 0.0)

    def loop(self, handle, step, timeout: float, what: str):
        """Appelle step() a la cadence du noeud jusqu'a ce qu'il rende True."""
        period = 1.0 / self.rate
        deadline = self.now() + timeout
        while rclpy.ok():
            if handle.is_cancel_requested:
                raise Canceled
            if self.now() > deadline:
                raise Failure(RES.FAILED_TO_CONTROL, f'{what} : pas termine en {timeout:.0f} s')
            if step():
                return
            time.sleep(period)

    def stage(self, handle, target: Pose2D, feedback=None):
        stager = docking.Stager(target, self.g)

        def step():
            v, w, done = stager.step(self.robot_at())
            self.drive(v, w)
            if feedback:
                handle.publish_feedback(feedback)
            return done
        self.loop(handle, step, self.stage_timeout, 'mise en place')
        self.stop()

    def run_dock(self, handle):
        result = DockRobot.Result()
        feedback = DockRobot.Feedback()
        start = self.now()
        with self.busy:
            try:
                if self.robot_at() is None:
                    raise Failure(RES.UNKNOWN, 'pas d odometrie')
                for attempt in range(self.max_retries + 1):
                    feedback.num_retries = attempt
                    try:
                        self.dock_once(handle, feedback, retry=attempt > 0)
                    except Failure as why:
                        # Cube perdu : on repart en mise en place sur sa derniere
                        # pose connue, d'ou il redevient visible.
                        if why.code != RES.FAILED_TO_DETECT_DOCK or attempt == self.max_retries:
                            raise
                        self.stop()
                        self.get_logger().warn(f'{why}, reprise')
                        continue
                    robot, target = self.robot_at(), self.fresh_target() or self.target
                    if docking.arrived_ok(robot, target, self.g):
                        along, lat, yaw = docking.errors(robot, target)
                        self.get_logger().info(
                            f'arrive : {along * 1000:+.0f} mm sur l axe, {lat * 1000:+.0f} mm lateral, '
                            f'{math.degrees(yaw):+.1f} deg, {self.now() - start:.1f} s, '
                            f'{attempt} reprise(s)')
                        result.success, result.num_retries = True, attempt
                        handle.succeed()
                        return result
                    along, lat, yaw = docking.errors(robot, target)
                    self.get_logger().warn(f'arrivee hors tolerance ({lat * 1000:+.0f} mm, '
                                           f'{math.degrees(yaw):+.1f} deg), reprise')
                raise Failure(RES.FAILED_TO_CONTROL, f'hors tolerance apres {self.max_retries} reprises')
            except Canceled:
                self.stop()
                handle.canceled()
                return result
            except Failure as why:
                self.stop()
                self.get_logger().error(f'docking : {why}')
                result.success, result.error_code, result.error_msg = False, why.code, str(why)
                result.num_retries = feedback.num_retries
                handle.abort()
                return result

    def dock_once(self, handle, feedback, retry: bool):
        target = self.fresh_target() or (self.target if retry else None)
        if target is None:
            raise Failure(RES.FAILED_TO_DETECT_DOCK, 'cube non vu')
        if docking.staging_needed(self.robot_at(), target, self.g):
            feedback.state = FB.NAV_TO_STAGING_POSE
            handle.publish_feedback(feedback)
            self.stage(handle, target.ahead(-self.g.staging), feedback)
        # Le cube a pu sortir du champ pendant les pivots : on attend qu'il
        # revienne avant de s'asservir dessus.
        feedback.state = FB.INITIAL_PERCEPTION
        handle.publish_feedback(feedback)
        self.loop(handle, lambda: self.fresh_target() is not None, 3.0, 'cube non revu apres la mise en place')
        feedback.state = FB.CONTROLLING

        def step():
            target = self.fresh_target()
            if target is None:
                raise Failure(RES.FAILED_TO_DETECT_DOCK, 'cube perdu pendant l approche')
            v, w, done = docking.approach(self.robot_at(), target, self.g)
            self.drive(v, w)
            handle.publish_feedback(feedback)
            return done
        self.loop(handle, step, 30.0, 'approche')
        self.stop()
        time.sleep(0.3)    # laisser le robot s'immobiliser avant de juger l'arrivee

    def run_undock(self, handle):
        result = UndockRobot.Result()
        with self.busy:
            try:
                robot = self.robot_at()
                if robot is None:
                    raise Failure(UndockRobot.Result.UNKNOWN, 'pas d odometrie')
                self.stage(handle, robot.ahead(-self.undock_distance))
                result.success = True
                handle.succeed()
            except Canceled:
                self.stop()
                handle.canceled()
            except Failure as why:
                self.stop()
                result.success, result.error_code, result.error_msg = False, why.code, str(why)
                handle.abort()
        return result


def main():
    rclpy.init()
    node = CubeDocking()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
