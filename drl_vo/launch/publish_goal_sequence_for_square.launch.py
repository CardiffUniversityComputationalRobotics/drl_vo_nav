from launch import LaunchDescription
from launch_ros.actions import Node

P_SEQ = [
    4, 5.5, 0, 4.5, 14, 0, 9, 17.5, 0, 17, 21.5, 0, 20, 15, 0,
    22, 8, 0, 12.7, 7.2, 0, 3, 8.6, 0, 8.6, 16.6, 0, 18, 20.7, 0,
    20.3, 18, 0, 23, 9, 0, 18, 6.5, 0, 7, 6, 0, 2, 12, 0,
    11, 14.3, 0, 18, 20, 0, 22, 12.7, 0, 17, 6.6, 0, 9, 17, 0,
    3.5, 19, 0, 19, 20, 0, 23, 11, 0, 18, 6.6, 0, 8, 16, 0,
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
