from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='drl_vo',
                executable='pure_pursuit',
                name='pure_pursuit',
                output='screen',
                remappings=[('path', '/plan')],
                parameters=[{'rate': 20.0}],
            ),
            Node(package='drl_vo', executable='cnn_data_pub', name='cnn_data_pub'),
            Node(package='drl_vo', executable='robot_pose_pub', name='robot_pose_pub'),
            Node(
                package='drl_vo',
                executable='track_ped_pub',
                name='track_ped_pub',
                output='screen',
            ),
            Node(
                package='drl_vo',
                executable='goal_visualize',
                name='goal_visualize',
                output='screen',
            ),
        ]
    )
