from launch import LaunchDescription
from launch_ros.actions import Node

P_SEQ = [
    6, 0, 0, 7, 7, 0, 10, 9.5, 0, 17, 9.5, 0, 24, 7.5, 0,
    18, 6.5, 0, 17, 1, 0, 17, -5, 0, 22, -6, 0, 27, -4, 0,
    27, 1.5, 0, 23, 3.4, 0, 18, 2.3, 0, 19, -3, 0, 22, -6.4, 0,
    23, 2, 0, 18, 6.7, 0, 15.5, 10, 0, 13, 6, 0, 11, 9.5, 0,
    6, 6.5, 0, 6, 0, 0, 13, -1, 0, 19, -0.5, 0, 20.5, 6, 0,
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
