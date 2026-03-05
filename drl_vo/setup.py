from glob import glob
from setuptools import find_packages, setup

package_name = 'drl_vo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/model', glob('model/*.zip')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xzt',
    maintainer_email='xzt@todo.todo',
    description='Deep Reinforcement Learning based Visual Obstacle Avoidance package for ROS 2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cnn_data_pub = drl_vo.cnn_data_pub:main',
            'cmd_vel_pub = drl_vo.cmd_vel_pub:main',
            'track_ped_pub = drl_vo.track_ped_pub:main',
            'robot_pose_pub = drl_vo.robot_pose_pub:main',
            'pure_pursuit = drl_vo.pure_pursuit:main',
            'goal_visualize = drl_vo.goal_visualize:main',
            'drl_vo_inference = drl_vo.drl_vo_inference:main',
            'drl_vo_train = drl_vo.drl_vo_train:main',
            'publish_goal_sequence = drl_vo.publish_goal_sequence:main',
        ],
    },
)
