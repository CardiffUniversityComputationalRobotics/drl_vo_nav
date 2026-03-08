

import threading

import numpy as np
import rclpy
from esc_move_base_msgs.msg import Path2D
from geometry_msgs.msg import Point
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_inverse,
    quaternion_multiply,
)


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.lookahead = 1.0
        self.declare_parameter('rate', 20.0)
        self.rate = float(self.get_parameter('rate').value)
        self.rate = 100
        self.goal_margin = 0.4
        self.waypoint_tolerance = 1.0
        self.final_goal_tolerance = 0.4

        self.wheel_base = 0.23
        self.wheel_radius = 0.1
        self.v_max = 0.5
        self.w_max = 5.0

        self.path = None
        self.active_segment_idx = 0
        self.lock = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Path2D, '/esc_move_base_planner/solution_path', self.path_callback, 10)
        self.cnn_goal_pub = self.create_publisher(Point, 'cnn_goal', 10)
        self.final_goal_pub = self.create_publisher(Point, 'final_goal', 10)

        self.timer = None

    def path_callback(self, msg: Path2D) -> None:
        self.get_logger().debug('PurePursuit: Got path')
        with self.lock:
            self.path = msg
            self.active_segment_idx = 0

        if self.timer is None:
            self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def _waypoint_xy(self, idx):
        waypoint = self.path.waypoints[idx]
        return np.array([waypoint.x, waypoint.y], dtype=np.float32)

    def _lookup_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            self.get_logger().warn('Could not get robot pose')
            return None

        trans = transform.transform.translation
        rot = transform.transform.rotation
        x = np.array([trans.x, trans.y], dtype=np.float32)
        quat = [rot.x, rot.y, rot.z, rot.w]
        (_, _, theta) = euler_from_quaternion(quat)
        return x, theta, quat

    def _advance_active_segment(self, x):
        waypoint_count = len(self.path.waypoints)
        if waypoint_count < 2:
            self.active_segment_idx = 0
            return

        seg_max = waypoint_count - 2
        seg = int(np.clip(self.active_segment_idx, 0, seg_max))

        while seg < seg_max:
            next_wp = self._waypoint_xy(seg + 1)
            if np.linalg.norm(x - next_wp) <= self.waypoint_tolerance:
                seg += 1
            else:
                break

        self.active_segment_idx = seg

    def _goal_on_active_segment(self, x):
        waypoint_count = len(self.path.waypoints)
        if waypoint_count == 1:
            return self._waypoint_xy(0)

        seg = int(np.clip(self.active_segment_idx, 0, waypoint_count - 2))
        p_start = self._waypoint_xy(seg)
        p_end = self._waypoint_xy(seg + 1)
        v = p_end - p_start
        length_seg = np.linalg.norm(v)
        if length_seg <= 1e-9:
            return p_end

        v = v / length_seg
        dist_projected = np.dot(x - p_start, v)
        dist_projected = float(np.clip(dist_projected, 0.0, length_seg))
        pt = p_start + dist_projected * v
        step = min(self.lookahead, length_seg - dist_projected)
        return pt + step * v

    def _select_goal(self, x):
        end_goal_pos = [self.path.waypoints[-1].x, self.path.waypoints[-1].y]
        end_goal_xy = np.array(end_goal_pos, dtype=np.float32)
        end_goal_rot = quaternion_from_euler(0.0, 0.0, self.path.waypoints[-1].theta)

        if len(self.path.waypoints) == 1:
            return end_goal_xy, end_goal_pos, end_goal_rot

        if np.linalg.norm(x - end_goal_xy) <= self.final_goal_tolerance:
            return end_goal_xy, end_goal_pos, end_goal_rot

        goal = self._goal_on_active_segment(x)
        return goal, end_goal_pos, end_goal_rot

    def timer_callback(self):
        with self.lock:
            pose = self._lookup_pose()
            if pose is None:
                return
            x, theta, rot = pose
            if np.isnan(x[0]):
                return

            if self.path is None or len(self.path.waypoints) == 0:
                return

            self._advance_active_segment(x)
            goal, end_goal_pos, end_goal_rot = self._select_goal(x)
            if goal is None or end_goal_pos is None:
                return

        map_t_robot = np.array([
            [np.cos(theta), -np.sin(theta), x[0]],
            [np.sin(theta), np.cos(theta), x[1]],
            [0.0, 0.0, 1.0],
        ])

        goal_local = np.matmul(np.linalg.inv(map_t_robot), np.array([[goal[0]], [goal[1]], [1.0]]))[0:2]
        relative_goal = np.matmul(
            np.linalg.inv(map_t_robot),
            np.array([[end_goal_pos[0]], [end_goal_pos[1]], [1.0]]),
        )

        orientation_to_target = quaternion_multiply(end_goal_rot, quaternion_inverse(rot))
        yaw = euler_from_quaternion(orientation_to_target)[2]

        cnn_goal = Point()
        cnn_goal.x = float(goal_local[0])
        cnn_goal.y = float(goal_local[1])
        cnn_goal.z = 0.0
        if not np.isnan(cnn_goal.x) and not np.isnan(cnn_goal.y):
            self.cnn_goal_pub.publish(cnn_goal)

        final_goal = Point()
        final_goal.x = float(relative_goal[0])
        final_goal.y = float(relative_goal[1])
        final_goal.z = float(yaw)
        if not np.isnan(final_goal.x) and not np.isnan(final_goal.y):
            self.final_goal_pub.publish(final_goal)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
