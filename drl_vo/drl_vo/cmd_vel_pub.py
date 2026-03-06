

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class VelSwitchNode(Node):
    def __init__(self):
        super().__init__('mix_cmd_vel')
        self.create_subscription(Twist, '/drl_cmd_vel', self.drl_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def drl_callback(self, drl_vel_msg: Twist) -> None:
        cmd_vel = Twist()
        cmd_vel.linear.x = drl_vel_msg.linear.x
        cmd_vel.angular.z = drl_vel_msg.angular.z
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = VelSwitchNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
