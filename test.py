#!/usr/bin/env python

import gym
import gym_op3

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

env = gym.make('gym_op3:op3-v0')

while True:

    env.step()