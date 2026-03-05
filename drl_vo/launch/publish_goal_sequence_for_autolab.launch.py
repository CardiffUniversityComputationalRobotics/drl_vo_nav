from launch import LaunchDescription
from launch_ros.actions import Node

P_SEQ = [
    0, 6, 0, 3, 11.8, 0, 1.6, 5.8, 0, 9.2, 8.2, 0, 14.9, 9, 0,
    8, 8, 0, 6.4, 2.9, 0, 7.8, -0.7, 0, 14.2, 1.8, 0, 20.2, -0.4, 0,
    12.8, -1.6, 0, 12.1, 5.5, 0, 9.7, 0.7, 0, 3.4, 1.9, 0, -0.4, 4.5, 0,
    0, 8.6, 0, 4.7, 6.5, 0, 6.1, 11.6, 0, 5.3, 6, 0, 9, 1.7, 0,
    0.7, -0.5, 0, 8.2, -0.4, 0, 14.5, 2, 0, 8.3, 2, 0, 9.3, 7.8, 0,
]
YEA_SEQ = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
]


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='drl_vo',
                executable='publish_goal_sequence',
                name='publish_goal',
                output='screen',
                parameters=[{'p_seq': P_SEQ, 'yea_seq': YEA_SEQ}],
            )
        ]
    )
