from launch import LaunchDescription
from launch_ros.actions import Node

P_SEQ = [
    3, 4, 0, 3, 8, 0, 5, 15, 0, 14, 18, 0, 18, 20, 0,
    19, 15, 0, 22, 11, 0, 16, 12, 0, 8, 10, 0, 3, 9, 0,
    2, 13, 0, 5, 16, 0, 12, 18, 0, 18, 19, 0, 14, 17, 0,
    11, 11, 0, 17, 12, 0, 19, 11, 0, 22, 11, 0, 19, 16, 0,
    20, 19, 0, 12, 19, 0, 8, 16, 0, 4, 13, 0, 3, 8, 0,
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
