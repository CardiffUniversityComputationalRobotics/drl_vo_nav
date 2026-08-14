

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class VelSwitchNode(Node):
    def __init__(self):
        super().__init__('mix_cmd_vel')
        self.goal_available = True

        self.create_subscription(Twist, '/drl_cmd_vel', self.drl_callback, 10)
        self.create_subscription(Bool, '/goal_available', self.goal_available_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def goal_available_callback(self, msg: Bool) -> None:
        was_goal_available = self.goal_available
        self.goal_available = bool(msg.data)

        if was_goal_available and not self.goal_available:
            self.cmd_vel_pub.publish(Twist())

    def drl_callback(self, drl_vel_msg: Twist) -> None:
        cmd_vel = Twist()

        if self.goal_available:
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
