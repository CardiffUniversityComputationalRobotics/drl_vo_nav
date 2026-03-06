

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from kobuki_ros_interfaces.msg import BumperEvent
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from tf_transformations import quaternion_from_euler


GOAL_NUM = 3


class MoveBaseSeqNode(Node):
    def __init__(self):
        super().__init__('publish_goal')

        self.success_num = 0
        self.total_time = 0.0
        self.start_time = 0.0
        self.end_time = 0.0

        self.total_distance = 0.0
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.odom_start = True

        self.bump_flag = False

        self.declare_parameter('p_seq', [])
        self.declare_parameter('yea_seq', [])
        points_seq = list(self.get_parameter('p_seq').value)
        yaw_seq = list(self.get_parameter('yea_seq').value)

        self.create_subscription(BumperEvent, '/mobile_base/events/bumper', self.bumper_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.pose_seq = []
        points = [points_seq[i : i + GOAL_NUM] for i in range(0, len(points_seq), GOAL_NUM)]
        self.get_logger().info(str(points))

        for i, point in enumerate(points):
            yaw_deg = yaw_seq[i] if i < len(yaw_seq) else 0.0
            q = Quaternion(*quaternion_from_euler(0.0, 0.0, yaw_deg * math.pi / 180.0, axes='sxyz'))
            self.pose_seq.append(Pose(Point(*point), q))

        self.goal_cnt = 0
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self.client.wait_for_server()
        self.get_logger().info('Connected to navigate_to_pose action server')
        self.get_logger().info('Starting goals achievements ...')

        self.send_next_goal()

    def bumper_callback(self, bumper_msg: BumperEvent) -> None:
        if bumper_msg.state == BumperEvent.PRESSED:
            self.bump_flag = True
        self.get_logger().info(f'Bumper Event: {bumper_msg.bumper}')

    def odom_callback(self, odom_msg: Odometry) -> None:
        if self.odom_start:
            self.previous_x = odom_msg.pose.pose.position.x
            self.previous_y = odom_msg.pose.pose.position.y

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        d_increment = math.hypot((x - self.previous_x), (y - self.previous_y))
        self.total_distance += d_increment
        self.previous_x = x
        self.previous_y = y
        self.odom_start = False

    def feedback_cb(self, feedback_msg) -> None:
        del feedback_msg
        self.get_logger().info(f'Feedback for goal pose {self.goal_cnt + 1} received')

    def done_cb(self, future) -> None:
        result = future.result()
        status = result.status
        self.goal_cnt += 1

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal pose {self.goal_cnt} reached')
            if not self.bump_flag:
                self.success_num += 1
            self.bump_flag = False

            self.end_time = self.get_clock().now().nanoseconds / 1e9
            self.total_time = self.end_time - self.start_time

            self.get_logger().info(
                f'Success Number: {self.success_num} in total number {self.goal_cnt}'
            )
            self.get_logger().info(f'Total Running Time: {self.total_time} secs')
            self.get_logger().info(f'Total Trajectory Length: {self.total_distance} m')

            if self.goal_cnt < len(self.pose_seq):
                self.send_next_goal()
            else:
                self.get_logger().info('Final goal pose reached!')
                rclpy.shutdown()
            return

        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'Goal pose {self.goal_cnt} was aborted by the Action Server')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Goal pose {self.goal_cnt} was canceled')
        else:
            self.get_logger().warn(f'Goal pose {self.goal_cnt} ended with status {status}')

        self.get_logger().info(
            f'Success Number: {self.success_num} in total number {self.goal_cnt}'
        )
        self.get_logger().info(f'Total Running Time: {self.total_time} secs')
        self.get_logger().info(f'Total Trajectory Length: {self.total_distance} m')
        rclpy.shutdown()

    def goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by navigate_to_pose action server')
            rclpy.shutdown()
            return

        self.get_logger().info(
            f'Goal pose {self.goal_cnt + 1} is now being processed by the Action Server...'
        )
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.done_cb)

    def send_next_goal(self) -> None:
        if self.goal_cnt >= len(self.pose_seq):
            self.get_logger().info('Final goal pose reached!')
            rclpy.shutdown()
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = self.pose_seq[self.goal_cnt]

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f'Sending goal pose {self.goal_cnt + 1} to Action Server')
        self.get_logger().info(str(self.pose_seq[self.goal_cnt]))

        self._send_goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_cb)


def main(args=None):
    rclpy.init(args=args)
    node = MoveBaseSeqNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
