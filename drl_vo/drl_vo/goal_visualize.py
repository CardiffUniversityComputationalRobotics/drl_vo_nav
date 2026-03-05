#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker


class GoalVisualizeNode(Node):
    def __init__(self):
        super().__init__('goal_vis')

        self.declare_parameter('goal_topic', '/goal_pose')
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value

        self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 10)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_vis_pub = self.create_publisher(Marker, 'goal_markers', qos)

    def goal_callback(self, goal_msg: PoseStamped) -> None:
        goal_marker = Marker()
        goal_marker.header.frame_id = 'map'
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose = goal_msg.pose
        goal_marker.scale.x = 1.8
        goal_marker.scale.y = 1.8
        goal_marker.scale.z = 1.8
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 0.5
        self.goal_vis_pub.publish(goal_marker)


def main(args=None):
    rclpy.init(args=args)
    node = GoalVisualizeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
