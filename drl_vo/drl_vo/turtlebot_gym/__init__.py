from gym.envs.registration import register

register(
    id='drl-nav-v0',
    entry_point='drl_vo.turtlebot_gym.envs:DRLNavEnv',
)
