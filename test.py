#!/usr/bin/env python

import numpy as np
import gym
import gym_op3
import pybullet as p

env = gym.make('gym_op3:op3-v0')

print('# joints:', env.num_joints)

joint_infos = [p.getJointInfo(env.robot_id, i) for i in range(env.num_joints)]

test_joint = 9

info = joint_infos[test_joint]
print('-----------------------')
print(info[1].decode('utf8')) # jointName
print('  index:', info[0])
print('  damping:', info[6])
print('  friction:', info[7])
print('  lower limit:', info[8])
print('  upper limit:', info[9])
print('  max force:', info[10])
print('  max velocity:', info[11])
print('  link name:', info[12].decode('utf8'))

random_action = True

i = 0
while True:
    if random_action:
        action = 2*np.random.random((12,)) - 1
    else:
        action = np.zeros(12)
        action[test_joint] = min(1.0, i * 0.01)
    obs, reward, done, _ = env.step(action)
    if done:
        env.reset()
        i = -1
    i += 1
    