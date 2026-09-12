#!/usr/bin/env python3
"""Odometrie du robot : distance des roues, cap du gyromètre.

    /diff_drive_controller/odom  (roues seules, publiee par ros2_control)
    /imu/data_raw                (BNO055 via le Pico)
        -> /odom  +  repere odom -> base_footprint

C'est ce noeud qui publie le repere odom : diff_drive_controller ne le fait
plus (enable_odom_tf: false dans controllers.yaml), sinon deux sources se
disputeraient la meme transformation. Son odometrie roues reste publiee et
sert de temoin : comparer /odom et /diff_drive_controller/odom montre
exactement ce que le gyromètre apporte.

Le calcul est dans alidade_bringup/odometry.py, teste sans ROS ; ici on ne
fait que brancher les topics, surveiller la fraicheur de l'IMU et publier.
"""
import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from alidade_bringup import odometry


class GyroOdom(Node):
    def __init__(self):
        super().__init__('gyro_odom')
        p = lambda name, default: self.declare_parameter(name, default).value
        gains = odometry.Gains(
            scale=p('gyro_scale', 1.0),
            still_v=p('still_v', 0.005),
            still_w=p('still_w', 0.02),
            still_gyro=math.radians(p('still_gyro_deg', 2.0)),
            bias_tau=p('bias_tau', 2.0),
            bias_max=math.radians(p('bias_max_deg', 1.0)))
        self.odom = odometry.Odometry(gains=gains)
        self.imu_timeout = p('imu_timeout', 0.15)
        self.odom_frame = p('odom_frame', 'odom')
        self.base_frame = p('base_frame', 'base_footprint')
        # Variances de cap : le gyromètre vaut cent fois mieux que les roues
        # (0.1 deg/s de bruit contre 1 a 2 % sur chaque rotation).
        self.yaw_var = (p('yaw_std_deg', 0.3), p('yaw_std_fallback_deg', 5.0))
        self.xy_var = p('xy_std', 0.02) ** 2

        self.imu = None          # (instant, w_z) du dernier echantillon exploitable
        self.last_stamp = None
        self.warned = 0
        self.pub = self.create_publisher(OdometryMsg, '/odom', 20)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(OdometryMsg, '/diff_drive_controller/odom', self.on_wheels, 20)
        self.get_logger().info(
            f'cap du gyromètre (echelle {gains.scale:.4f}), distance des roues ; '
            f'repli sur les roues au-dela de {self.imu_timeout * 1000:.0f} ms sans IMU')

    def on_imu(self, msg):
        g, a = msg.angular_velocity, msg.linear_acceleration
        if odometry.usable((g.x, g.y, g.z), (a.x, a.y, a.z)):
            self.imu = (self.now(), g.z)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_wheels(self, msg):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.last_stamp is None:
            self.last_stamp = stamp
            return
        dt, self.last_stamp = stamp - self.last_stamp, stamp
        fresh = self.imu is not None and self.now() - self.imu[0] <= self.imu_timeout
        v, w_wheel = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        before = self.odom.fallbacks
        self.odom.step(dt, v, w_wheel, self.imu[1] if fresh else None)
        if self.odom.fallbacks > before:
            self.report_fallback()
        self.publish(msg.header.stamp, v, w_wheel)

    def report_fallback(self):
        """Une IMU muette n'est pas anodine : on le dit, sans noyer le journal."""
        n = self.odom.fallbacks
        if n == 1 or n % 250 == 0:      # 250 pas = 5 s
            self.get_logger().warn(f'IMU indisponible, cap pris sur les roues ({n} pas)')

    def publish(self, stamp, v, w_wheel):
        o = self.odom
        gyro = o.source == odometry.GYRO
        msg = OdometryMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = o.x, o.y
        msg.pose.pose.orientation = Quaternion(z=math.sin(o.yaw / 2), w=math.cos(o.yaw / 2))
        yaw_var = math.radians(self.yaw_var[0 if gyro else 1]) ** 2
        msg.pose.covariance[0] = msg.pose.covariance[7] = self.xy_var
        msg.pose.covariance[35] = yaw_var
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w_wheel if not gyro else (self.imu[1] - o.bias) * o.gains.scale
        msg.twist.covariance[0] = self.xy_var
        msg.twist.covariance[35] = yaw_var
        self.pub.publish(msg)

        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = self.base_frame
        t.transform.translation.x, t.transform.translation.y = o.x, o.y
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = GyroOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
