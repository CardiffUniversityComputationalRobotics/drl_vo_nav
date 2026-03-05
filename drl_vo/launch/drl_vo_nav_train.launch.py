from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_file = LaunchConfiguration('model_file')
    log_dir = LaunchConfiguration('log_dir')
    rviz = LaunchConfiguration('rviz')

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
            DeclareLaunchArgument('rviz', default_value='true'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('drl_vo'), 'launch', 'nav_cnn_data.launch.py'])
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('drl_vo'), 'launch', 'drl_vo_train.launch.py'])
                ),
                launch_arguments={'model_file': model_file, 'log_dir': log_dir}.items(),
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                condition=IfCondition(rviz),
            ),
        ]
    )
