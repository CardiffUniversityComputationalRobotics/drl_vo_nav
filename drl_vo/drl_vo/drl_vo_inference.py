#!/usr/bin/env python3

import numpy as np
import numpy.matlib
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from cnn_msgs.msg import CNNData
from stable_baselines3 import PPO

from .custom_cnn_full import CustomCNN


policy_kwargs = dict(
    features_extractor_class=CustomCNN,
    features_extractor_kwargs=dict(features_dim=256),
)


class DrlInferenceNode(Node):
    def __init__(self):
        super().__init__('drl_inference')

        self.ped_pos = []
        self.scan = []
        self.goal = []

        self.declare_parameter('model_file', './model/drl_vo.zip')
        model_file = self.get_parameter('model_file').get_parameter_value().string_value
        self.model = PPO.load(model_file)
        self.get_logger().info(f'Loaded model: {model_file}')

        self.create_subscription(CNNData, '/cnn_data', self.cnn_data_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/drl_cmd_vel', 10)

    def cnn_data_callback(self, cnn_data_msg: CNNData) -> None:
        self.ped_pos = cnn_data_msg.ped_pos_map
        self.scan = cnn_data_msg.scan
        self.goal = cnn_data_msg.goal_cart

        cmd_vel = Twist()

        scan = np.array(self.scan[-540:-180])
        scan = scan[scan != 0]
        min_scan_dist = np.amin(scan) if scan.size != 0 else 10.0

        if np.linalg.norm(self.goal) <= 0.9:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        elif min_scan_dist <= 0.4:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.7
        else:
            v_min, v_max = -2.0, 2.0
            self.ped_pos = np.array(self.ped_pos, dtype=np.float32)
            self.ped_pos = 2 * (self.ped_pos - v_min) / (v_max - v_min) - 1

            temp = np.array(self.scan, dtype=np.float32)
            scan_avg = np.zeros((20, 80), dtype=np.float32)
            for n in range(10):
                scan_tmp = temp[n * 720 : (n + 1) * 720]
                for i in range(80):
                    scan_avg[2 * n, i] = np.min(scan_tmp[i * 9 : (i + 1) * 9])
                    scan_avg[2 * n + 1, i] = np.mean(scan_tmp[i * 9 : (i + 1) * 9])

            scan_avg = scan_avg.reshape(1600)
            scan_avg_map = np.matlib.repmat(scan_avg, 1, 4)
            self.scan = scan_avg_map.reshape(6400)
            s_min, s_max = 0.0, 30.0
            self.scan = 2 * (self.scan - s_min) / (s_max - s_min) - 1

            g_min, g_max = -2.0, 2.0
            goal_original = np.array(self.goal, dtype=np.float32)
            self.goal = 2 * (goal_original - g_min) / (g_max - g_min) - 1

            observation = np.concatenate((self.ped_pos, self.scan, self.goal), axis=None)
            action, _states = self.model.predict(observation)

            vx_min, vx_max = 0.0, 0.5
            vz_min, vz_max = -2.0, 2.0
            cmd_vel.linear.x = (action[0] + 1) * (vx_max - vx_min) / 2 + vx_min
            cmd_vel.angular.z = (action[1] + 1) * (vz_max - vz_min) / 2 + vz_min

        if not np.isnan(cmd_vel.linear.x) and not np.isnan(cmd_vel.angular.z):
            self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = DrlInferenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
