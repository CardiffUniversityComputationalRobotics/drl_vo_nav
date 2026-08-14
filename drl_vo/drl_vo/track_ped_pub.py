import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from pedsim_msgs.msg import TrackedPerson, TrackedPersons
from rclpy.node import Node
from tf_transformations import euler_from_quaternion


class TrackPedNode(Node):
    def __init__(self):
        super().__init__('track_ped')

        self.create_subscription(
            TrackedPersons,
            '/pedsim_visualizer/tracked_persons',
            self.ped_callback,
            10,
        )
        self.create_subscription(Odometry, '/odom_groundtruth', self.odom_callback, 10)
        self.track_ped_pub = self.create_publisher(TrackedPersons, '/track_ped', 10)

        self._robot_pose = None
        self._warned_no_robot_pose = False

    def odom_callback(self, odom_msg: Odometry) -> None:
        pose = odom_msg.pose.pose
        robot_q = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        (_, _, yaw) = euler_from_quaternion(robot_q)
        self._robot_pose = np.array([pose.position.x, pose.position.y, yaw], dtype=np.float32)

    def get_robot_pose(self):
        if self._robot_pose is None:
            if not self._warned_no_robot_pose:
                self.get_logger().warn('No robot pose received yet on /odom_groundtruth')
                self._warned_no_robot_pose = True
            return None

        return self._robot_pose.copy()

    def ped_callback(self, peds_msg: TrackedPersons) -> None:
        robot_pos = self.get_robot_pose()
        if robot_pos is None:
            return

        map_r_robot = np.array(
            [
                [np.cos(robot_pos[2]), -np.sin(robot_pos[2])],
                [np.sin(robot_pos[2]), np.cos(robot_pos[2])],
            ],
            dtype=np.float32,
        )
        map_t_robot = np.array(
            [
                [np.cos(robot_pos[2]), -np.sin(robot_pos[2]), robot_pos[0]],
                [np.sin(robot_pos[2]), np.cos(robot_pos[2]), robot_pos[1]],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        robot_r_map = np.linalg.inv(map_r_robot)
        robot_t_map = np.linalg.inv(map_t_robot)

        tracked_peds = TrackedPersons()
        tracked_peds.header.frame_id = 'base_footprint'
        tracked_peds.header.stamp = self.get_clock().now().to_msg()

        for ped in peds_msg.tracks:
            ped_pos = np.array([ped.pose.pose.position.x, ped.pose.pose.position.y, 1.0], dtype=np.float32)
            ped_vel = np.array([ped.twist.twist.linear.x, ped.twist.twist.linear.y], dtype=np.float32)
            ped_pos_in_robot = np.matmul(robot_t_map, ped_pos.T)
            ped_vel_in_robot = np.matmul(robot_r_map, ped_vel.T)

            tracked_ped = TrackedPerson()
            tracked_ped = ped
            tracked_ped.pose.pose.position.x = float(ped_pos_in_robot[0])
            tracked_ped.pose.pose.position.y = float(ped_pos_in_robot[1])
            tracked_ped.twist.twist.linear.x = float(ped_vel_in_robot[0])
            tracked_ped.twist.twist.linear.y = float(ped_vel_in_robot[1])
            tracked_peds.tracks.append(tracked_ped)

        self.track_ped_pub.publish(tracked_peds)


def main(args=None):
    rclpy.init(args=args)
    node = TrackPedNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
