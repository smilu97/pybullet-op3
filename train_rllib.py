#!/usr/bin/env python

from ray import tune
from ray.rllib.agents.ppo import PPOTrainer
from ray.rllib.agents.sac import SACTrainer
from ray.tune.registry import register_env
from ppo_config import config as ppo_config
from sac_config import config as sac_config

import gym
import gym_op3
import pybulletgym

def env_creator(env_config):
    return gym.make('gym_op3:op3-v0')

trainer = 'ppo'

config = ppo_config if trainer == 'ppo' else sac_config
t = PPOTrainer if trainer == 'ppo' else SACTrainer

register_env("py-op3-v0", env_creator)
config["env"] = "py-op3-v0"

tune.run(
    t,
    name='pyop3',
    resume=False,
    config=config,
    checkpoint_freq=30,
    checkpoint_at_end=True)
    
