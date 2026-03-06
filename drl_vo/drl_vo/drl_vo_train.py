
import os

import gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy

from drl_vo import ros2_compat as rospy
from drl_vo import turtlebot_gym  # noqa: F401
from drl_vo.custom_cnn_full import CustomCNN


class SaveOnBestTrainingRewardCallback(BaseCallback):
    def __init__(self, check_freq: int, log_dir: str, verbose=1):
        super().__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, 'best_model')
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            x, y = ts2xy(load_results(self.log_dir), 'timesteps')
            if len(x) > 0:
                mean_reward = np.mean(y[-100:])
                if self.verbose > 0:
                    print(f'Num timesteps: {self.num_timesteps}')
                    print(
                        f'Best mean reward: {self.best_mean_reward:.2f} '
                        f'- Last mean reward per episode: {mean_reward:.2f}'
                    )

                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                    if self.verbose > 0:
                        print(f'Saving new best model to {self.save_path}')
                    self.model.save(self.save_path)

        if self.n_calls % 20000 == 0:
            path = self.save_path + '_model' + str(self.n_calls)
            self.model.save(path)

        return True


def main():
    rospy.init_node('env_test', anonymous=True, log_level=rospy.WARN)

    log_dir = rospy.get_param('~log_dir', './runs/')
    os.makedirs(log_dir, exist_ok=True)

    env = gym.make('drl-nav-v0')
    env = Monitor(env, log_dir)
    env.reset()

    policy_kwargs = dict(
        features_extractor_class=CustomCNN,
        features_extractor_kwargs=dict(features_dim=256),
        net_arch=[dict(pi=[256], vf=[128])],
    )

    kwargs = {
        'tensorboard_log': log_dir,
        'verbose': 2,
        'n_epochs': 10,
        'n_steps': 512,
        'batch_size': 128,
        'learning_rate': 5e-5,
        'policy_kwargs': policy_kwargs,
    }
    model_file = rospy.get_param('~model_file', './model/drl_pre_train.zip')
    model = PPO.load(model_file, env=env, **kwargs)

    callback = SaveOnBestTrainingRewardCallback(check_freq=5000, log_dir=log_dir)
    model.learn(
        total_timesteps=2000000,
        log_interval=5,
        tb_log_name='drl_vo_policy',
        callback=callback,
        reset_num_timesteps=True,
    )

    model.save('drl_vo_model')
    print('Training finished.')
    env.close()


if __name__ == '__main__':
    main()
