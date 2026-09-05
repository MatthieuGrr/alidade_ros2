#!/usr/bin/env python3
"""Convertit le Twist de nav2 en TwistStamped pour diff_drive_controller.

nav2 (Jazzy) publie des geometry_msgs/msg/Twist en bout de chaine, tandis que
diff_drive_controller ne souscrit qu'a des TwistStamped, et aucun des deux
n'expose de parametre pour ceder. Ce relais fait le joint.

    ros2 run alidade_bringup cmd_vel_stamper.py \
        --ros-args -p input:=/cmd_vel -p output:=/diff_drive_controller/cmd_vel
"""
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelStamper(Node):
    def __init__(self):
        super().__init__('cmd_vel_stamper')
        self.declare_parameter('input', '/cmd_vel')
        self.declare_parameter('output', '/diff_drive_controller/cmd_vel')
        self.declare_parameter('frame_id', 'base_footprint')

        self.frame_id = self.get_parameter('frame_id').value
        source = self.get_parameter('input').value
        target = self.get_parameter('output').value

        self.pub = self.create_publisher(TwistStamped, target, 10)
        self.create_subscription(Twist, source, self.relay, 10)
        self.get_logger().info(f'{source} (Twist) -> {target} (TwistStamped)')

    def relay(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
