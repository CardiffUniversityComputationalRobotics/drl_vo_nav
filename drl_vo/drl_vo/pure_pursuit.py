#!/usr/bin/env python3

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_inverse, quaternion_multiply


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.lookahead = 2.0
        self.declare_parameter('rate', 20.0)
        self.rate = float(self.get_parameter('rate').value)
        self.goal_margin = 0.9

        self.wheel_base = 0.23
        self.wheel_radius = 0.025
        self.v_max = 0.5
        self.w_max = 5.0

        self.path = None
        self.lock = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Path, 'path', self.path_callback, 10)
        self.cnn_goal_pub = self.create_publisher(Point, 'cnn_goal', 10)
        self.final_goal_pub = self.create_publisher(Point, 'final_goal', 10)

        self.timer = None

    def path_callback(self, msg: Path) -> None:
        self.get_logger().debug('PurePursuit: Got path')
        with self.lock:
            self.path = msg

        if self.timer is None:
            self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

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
            for i in range(len(self.path.poses) - 1):
                pt, dist, s = self.find_closest_point(x, i)
                if dist < dist_min:
                    pt_min = pt
                    dist_min = dist
                    seg_min = s
        else:
            p_start = np.array([
                self.path.poses[seg].pose.position.x,
                self.path.poses[seg].pose.position.y,
            ])
            p_end = np.array([
                self.path.poses[seg + 1].pose.position.x,
                self.path.poses[seg + 1].pose.position.y,
            ])

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
            seg_max = len(self.path.poses) - 2
            p_end = np.array([
                self.path.poses[seg + 1].pose.position.x,
                self.path.poses[seg + 1].pose.position.y,
            ])
            dist_end = np.linalg.norm(x - p_end)

            while dist_end < self.lookahead and seg < seg_max:
                seg += 1
                p_end = np.array([
                    self.path.poses[seg + 1].pose.position.x,
                    self.path.poses[seg + 1].pose.position.y,
                ])
                dist_end = np.linalg.norm(x - p_end)

            if dist_end < self.lookahead:
                pt = np.array([
                    self.path.poses[seg_max + 1].pose.position.x,
                    self.path.poses[seg_max + 1].pose.position.y,
                ])
            else:
                pt, dist, seg = self.find_closest_point(x, seg)
                p_start = np.array([
                    self.path.poses[seg].pose.position.x,
                    self.path.poses[seg].pose.position.y,
                ])
                p_end = np.array([
                    self.path.poses[seg + 1].pose.position.x,
                    self.path.poses[seg + 1].pose.position.y,
                ])
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

        end_goal_pos = [self.path.poses[-1].pose.position.x, self.path.poses[-1].pose.position.y]
        end_goal_rot = [
            self.path.poses[-1].pose.orientation.x,
            self.path.poses[-1].pose.orientation.y,
            self.path.poses[-1].pose.orientation.z,
            self.path.poses[-1].pose.orientation.w,
        ]
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
