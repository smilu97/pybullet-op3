# author: smilu97

import gym
import xacro
import os
import pathlib
import pybullet as p
import numpy as np

from gym import error, spaces, utils
from gym.utils import seeding

from .xacro_urdf_env import XacroURDFEnv

# 0 : l_hip_yaw
# 1 : l_hip_roll
# 2 : l_hip_pitch
# 3 : l_knee
# 4 : l_ank_pitch
# 5 : l_ank_roll
# 6 : r_hip_yaw
# 7 : r_hip_roll
# 8 : r_hip_pitch
# 9 : r_knee
# 10: r_ank_pitch
# 11: r_ank_roll
# 12: l_sho_pitch
# 13: l_sho_roll
# 14: l_el
# 15: r_sho_pitch
# 16: r_sho_roll
# 17: r_el
# 18: head_pan
# 19: head_tilt

joint_indices = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
joint_lowers  = np.array([-1.70, -1.70, -1.70, -1.70, -1.70, -1.70, -1.70, -1.70, -1.70, -1.70, -1.70, -1.70])
joint_uppers  = np.array([ 1.70,  1.70,  1.70,  1.70,  1.70,  1.70,  1.70,  1.70,  1.70,  1.70,  1.70,  1.70])
joint_multis  = np.array([ 1.70,  1.70,  1.70,  1.70,   0.3,   0.3,  1.70,  1.70,  1.70,  1.70,   0.3,   0.3])
joint_bias    = np.array([ 0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00])

num_joints = 20

progress_bonus = 120.0
accel_bonus = 600.0
alive_bonus = 0.05
outroute_cost = -0.001
velocity_cost = -2.0
height_bonus = 0.1
effort_cost = -0.0001
stuck_cost = -0.0
class OP3Env(XacroURDFEnv):
    def __init__(self):
        current_dir = pathlib.Path(__file__).resolve().parent
        xacro_path = os.path.join(current_dir, 'urdf/robotis_op3.urdf.xacro')
        super().__init__(xacro_path, joint_indices, start_z=1.5)
        
        self.prev_x = 0

        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(len(joint_indices),), dtype=np.float64)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(2*num_joints+7,), dtype=np.float64)

    def step(self, action):
        action = np.array(action)
        action = joint_multis * action + joint_bias
        action = np.minimum(joint_uppers, action)
        action = np.maximum(joint_lowers, action)
        super().step(action)

        joint_states = p.getJointStates(self.robot_id, np.arange(num_joints))

        obs = np.zeros(num_joints * 2 + 7)
        for i in range(num_joints):
            obs[i             ] = joint_states[i][0]
            obs[i + num_joints] = joint_states[i][1]

        base = p.getLinkState(self.robot_id, 18)
        base_position = base[0]
        base_orientation = base[1]

        obs[num_joints * 2 + 0 : num_joints * 2 + 3] = base_position[:]
        obs[num_joints * 2 + 3 : num_joints * 2 + 7] = base_orientation[:]

        height = base_position[2] # body z-position

        dx = base_position[0] - self.prev_x
        self.prev_x = base_position[0]

        progress = dx * progress_bonus
        alive = alive_bonus
        outroute = abs(base_position[1]) * outroute_cost
        effort = max(-alive_bonus, sum([abs(x[3]) for x in joint_states]) * effort_cost)
        rewards = [progress, alive, outroute, effort]
        reward = sum(rewards)

        print('height:', height)

        return obs, reward, height < 0.6
        