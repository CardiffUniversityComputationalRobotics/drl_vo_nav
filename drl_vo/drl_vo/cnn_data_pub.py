

import numpy as np
import rclpy
from rclpy.node import Node

from cnn_msgs.msg import CNNData
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from pedsim_msgs.msg import TrackedPersons
from sensor_msgs.msg import LaserScan

NUM_TP = 10


class CnnDataNode(Node):
    def __init__(self):
        super().__init__('cnn_data')

        self.ped_pos_map = []
        self.scan = []
        self.scan_all = np.zeros(1080, dtype=np.float32)
        self.goal_cart = np.zeros(2, dtype=np.float32)
        self.vel = np.zeros(2, dtype=np.float32)

        self.ped_pos_map_tmp = np.zeros((2, 80, 80), dtype=np.float32)
        self.scan_tmp = np.zeros(720, dtype=np.float32)
        self.scan_all_tmp = np.zeros(1080, dtype=np.float32)

        self.create_subscription(TrackedPersons, '/track_ped', self.ped_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Point, '/cnn_goal', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cnn_data_pub = self.create_publisher(CNNData, '/cnn_data', 1)

        self.rate = 20.0
        self.ts_cnt = 0
        self.create_timer(1.0 / self.rate, self.timer_callback)

    def ped_callback(self, track_ped_msg: TrackedPersons) -> None:
        self.ped_pos_map_tmp = np.zeros((2, 80, 80), dtype=np.float32)
        if len(track_ped_msg.tracks) == 0:
            return

        for ped in track_ped_msg.tracks:
            x = ped.pose.pose.position.x
            y = ped.pose.pose.position.y
            vx = ped.twist.twist.linear.x
            vy = ped.twist.twist.linear.y
            if x >= 0.0 and x <= 20.0 and abs(y) <= 10.0:
                c = int(np.floor(-(y - 10.0) / 0.25))
                r = int(np.floor(x / 0.25))
                if r == 80:
                    r -= 1
                if c == 80:
                    c -= 1
                self.ped_pos_map_tmp[0, r, c] = vx
                self.ped_pos_map_tmp[1, r, c] = vy

    def scan_callback(self, laser_scan_msg: LaserScan) -> None:
        self.scan_tmp = np.zeros(720, dtype=np.float32)
        self.scan_all_tmp = np.zeros(1080, dtype=np.float32)
        scan_data = np.array(laser_scan_msg.ranges, dtype=np.float32)
        scan_data[np.isnan(scan_data)] = 0.0
        scan_data[np.isinf(scan_data)] = 0.0

        if scan_data.size >= 900:
            self.scan_tmp = scan_data[180:900]
        self.scan_all_tmp = scan_data

    def goal_callback(self, goal_msg: Point) -> None:
        self.goal_cart[0] = goal_msg.x
        self.goal_cart[1] = goal_msg.y

    def odom_callback(self, odom_msg: Odometry) -> None:

        vel_msg = odom_msg.twist.twist

        self.vel[0] = vel_msg.linear.x
        self.vel[1] = vel_msg.angular.z

    def timer_callback(self) -> None:
        self.ped_pos_map = self.ped_pos_map_tmp
        self.scan.append(self.scan_tmp.tolist())
        self.scan_all = self.scan_all_tmp

        self.ts_cnt += 1
        if self.ts_cnt == NUM_TP:
            cnn_data = CNNData()
            cnn_data.ped_pos_map = [
                float(val)
                for chan in self.ped_pos_map
                for row in chan
                for val in row
            ]
            cnn_data.scan = [float(val) for sub in self.scan for val in sub]
            cnn_data.scan_all = [float(x) for x in self.scan_all.tolist()]
            cnn_data.depth = []
            cnn_data.image_gray = []
            cnn_data.goal_cart = [float(x) for x in self.goal_cart.tolist()]
            cnn_data.goal_final_polar = []
            cnn_data.vel = [float(x) for x in self.vel.tolist()]
            self.cnn_data_pub.publish(cnn_data)

            self.ts_cnt = NUM_TP - 1
            self.scan = self.scan[1:NUM_TP]


def main(args=None):
    rclpy.init(args=args)
    node = CnnDataNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
