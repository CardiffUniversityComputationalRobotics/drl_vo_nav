#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__('robot_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
            )
        except TransformException:
            self.get_logger().warn('Could not get robot pose')
            return

        rob_pos = PoseStamped()
        rob_pos.header.stamp = self.get_clock().now().to_msg()
        rob_pos.header.frame_id = 'map'
        rob_pos.pose.position.x = transform.transform.translation.x
        rob_pos.pose.position.y = transform.transform.translation.y
        rob_pos.pose.position.z = transform.transform.translation.z
        rob_pos.pose.orientation = transform.transform.rotation
        self.robot_pose_pub.publish(rob_pos)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
