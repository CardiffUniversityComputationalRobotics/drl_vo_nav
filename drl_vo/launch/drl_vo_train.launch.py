from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_file = LaunchConfiguration('model_file')
    log_dir = LaunchConfiguration('log_dir')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'model_file',
                default_value=PathJoinSubstitution([FindPackageShare('drl_vo'), 'model', 'drl_pre_train.zip']),
            ),
            DeclareLaunchArgument(
                'log_dir',
                default_value=PathJoinSubstitution([FindPackageShare('drl_vo'), 'runs']),
            ),
            DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
            Node(
                package='drl_vo',
                executable='drl_vo_train',
                name='drl_vo_train',
                output='screen',
                parameters=[{'model_file': model_file, 'log_dir': log_dir}],
            ),
            Node(
                package='drl_vo',
                executable='cmd_vel_pub',
                name='mix_cmd_vel',
                output='screen',
                remappings=[('cmd_vel', cmd_vel_topic)],
            ),
        ]
    )
