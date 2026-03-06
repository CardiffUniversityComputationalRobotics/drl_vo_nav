import numpy as np
import rclpy
from gazebo_msgs.srv import GetEntityState
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
        self.get_state_client = self.create_client(GetEntityState, '/gazebo_spawner/get_entity_state')
        self.track_ped_pub = self.create_publisher(TrackedPersons, '/track_ped', 10)

        # Keep latest successful robot state and an in-flight request.
        self._robot_state = None
        self._pending_state_future = None

    def get_robot_states(self):
        # Consume completed request (if any).
        if self._pending_state_future is not None and self._pending_state_future.done():
            try:
                result = self._pending_state_future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'/gazebo_spawner/get_entity_state service call raised: {exc}')
            else:
                if result is None:
                    self.get_logger().warn('/gazebo_spawner/get_entity_state returned no result')
                elif not result.success:
                    self.get_logger().warn(
                        f"/gazebo_spawner/get_entity_state returned success=False: {result.status_message}"
                    )
                else:
                    self._robot_state = result.state
            self._pending_state_future = None

        # Issue a new async request when no request is in flight.
        if self._pending_state_future is None:
            if not self.get_state_client.service_is_ready():
                self.get_logger().warn('/gazebo_spawner/get_entity_state service not available')
                return self._robot_state

            request = GetEntityState.Request()
            request.name = 'reachy'
            request.reference_frame = 'world'
            self._pending_state_future = self.get_state_client.call_async(request)

        return self._robot_state

    def ped_callback(self, peds_msg: TrackedPersons) -> None:
        robot = self.get_robot_states()
        if robot is None:
            return

        robot_pos = np.zeros(3, dtype=np.float32)
        robot_pos[:2] = np.array([robot.pose.position.x, robot.pose.position.y], dtype=np.float32)
        robot_q = (
            robot.pose.orientation.x,
            robot.pose.orientation.y,
            robot.pose.orientation.z,
            robot.pose.orientation.w,
        )
        (_, _, robot_pos[2]) = euler_from_quaternion(robot_q)

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
