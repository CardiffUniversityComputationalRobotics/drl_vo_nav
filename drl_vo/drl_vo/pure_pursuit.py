

import threading

import numpy as np
import rclpy
from esc_move_base_msgs.msg import Path2D
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
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
        self.goal_margin = 0.9
        self.goal_reached_distance = 1.0

        self.global_goal = None

        self.wheel_base = 0.23
        self.wheel_radius = 0.1
        self.v_max = 0.5
        self.w_max = 5.0

        self.path = None
        self.robot_odom_pos = None
        self.lock = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Path2D, '/esc_move_base_planner/solution_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cnn_goal_pub = self.create_publisher(Point, 'cnn_goal', 10)
        self.final_goal_pub = self.create_publisher(Point, 'final_goal', 10)
        self.goal_achieved_pub = self.create_publisher(Bool, '/goal_reached', 10)

        self.timer = None

    def path_callback(self, msg: Path2D) -> None:
        self.get_logger().debug('PurePursuit: Got path')
        with self.lock:
            self.path = msg

        if self.timer is None:
            self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        
        if self.global_goal is None:
            self.global_goal = [self.path.waypoints[-1].x, self.path.waypoints[-1].y]



    def odom_callback(self, msg: Odometry) -> None:
        with self.lock:
            self.robot_odom_pos = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            ], dtype=np.float32)

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

    def find_closest_point(self, x, seg=-1):
        pt_min = np.array([np.nan, np.nan], dtype=np.float32)
        dist_min = np.inf
        seg_min = -1

        if self.path is None:
            self.get_logger().warn('Pure Pursuit: No path received yet')
            return pt_min, dist_min, seg_min

        if seg == -1:
            for i in range(len(self.path.waypoints) - 1):
                pt, dist, s = self.find_closest_point(x, i)
                if dist < dist_min:
                    pt_min = pt
                    dist_min = dist
                    seg_min = s
        else:
            p_start = self._waypoint_xy(seg)
            p_end = self._waypoint_xy(seg + 1)

            v = p_end - p_start
            length_seg = np.linalg.norm(v)
            if length_seg <= 1e-9:
                return p_start, np.linalg.norm(p_start - x), seg
            v = v / length_seg

            dist_projected = np.dot(x - p_start, v)
            if dist_projected < 0.0:
                pt_min = p_start
            elif dist_projected > length_seg:
                pt_min = p_end
            else:
                pt_min = p_start + dist_projected * v

            dist_min = np.linalg.norm(pt_min - x)
            seg_min = seg

        return pt_min, dist_min, seg_min

    def find_goal(self, x, pt, dist, seg):
        goal = None
        end_goal_pos = None
        end_goal_rot = None

        if dist > self.lookahead:
            goal = pt
        else:
            seg_max = len(self.path.waypoints) - 2
            p_end = self._waypoint_xy(seg + 1)
            dist_end = np.linalg.norm(x - p_end)

            while dist_end < self.lookahead and seg < seg_max:
                seg += 1
                p_end = self._waypoint_xy(seg + 1)
                dist_end = np.linalg.norm(x - p_end)

            if dist_end < self.lookahead:
                pt = self._waypoint_xy(seg_max + 1)
            else:
                pt, dist, seg = self.find_closest_point(x, seg)
                p_start = self._waypoint_xy(seg)
                p_end = self._waypoint_xy(seg + 1)
                v = p_end - p_start
                length_seg = np.linalg.norm(v)
                if length_seg <= 1e-9:
                    goal = p_end
                else:
                    v = v / length_seg
                    dist_projected_x = np.dot(x - pt, v)
                    dist_projected_y = np.linalg.norm(np.cross(x - pt, v))
                    inside = max(self.lookahead ** 2 - dist_projected_y ** 2, 0.0)
                    pt = pt + (np.sqrt(inside) + dist_projected_x) * v

            goal = pt

        end_goal_pos = [self.path.waypoints[-1].x, self.path.waypoints[-1].y]
        end_goal_rot = quaternion_from_euler(0.0, 0.0, self.path.waypoints[-1].theta)
        return goal, end_goal_pos, end_goal_rot

    def timer_callback(self):
        with self.lock:
            pose = self._lookup_pose()
            if pose is None:
                return
            x, theta, rot = pose
            if np.isnan(x[0]):
                return

            pt, dist, seg = self.find_closest_point(x)
            if np.isnan(pt).any():
                return

            goal, end_goal_pos, end_goal_rot = self.find_goal(x, pt, dist, seg)
            if goal is None or end_goal_pos is None:
                return

            robot_odom_pos = None if self.robot_odom_pos is None else self.robot_odom_pos.copy()

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

            if robot_odom_pos is not None and self.global_goal is not None:
                end_goal_xy = np.array(self.global_goal, dtype=np.float32)
                dist_to_final_waypoint = np.linalg.norm(robot_odom_pos - end_goal_xy)
                if dist_to_final_waypoint <= self.goal_reached_distance:
                    goal_reached = Bool()
                    goal_reached.data = True
                    self.goal_achieved_pub.publish(goal_reached)


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
