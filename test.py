#!/usr/bin/env python

import pybullet as p
import numpy as np
import time
import pybullet_data

import xacro
op3_doc = xacro.process_file('./urdf/robotis_op3.urdf.xacro')
op3_urdf = op3_doc.toprettyxml(indent='  ')

import os

op3Path = "tmp_op3.urdf"
if os.path.isfile(op3Path):
    os.unlink(op3Path)
open(op3Path, 'w').write(op3_urdf)
op3Path = os.path.join(os.path.dirname(__file__), op3Path)

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

p.configureDebugVisualizer(
    p.COV_ENABLE_GUI, 0
)

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)
p.setDefaultContactERP(0.9)
p.setPhysicsEngineParameter(fixedTimeStep=0.001, numSolverIterations=100, numSubSteps=5)

planeId = p.loadURDF("plane.urdf")

startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
flags = p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_IMPLICIT_CYLINDER
op3Id = p.loadURDF(op3Path, startPos, startOrientation, flags=flags)

numJoints = p.getNumJoints(op3Id)
print('# of joints:', numJoints)
jointInfos = [p.getJointInfo(op3Id, i) for i in range(numJoints)]
for info in jointInfos:
    print('-----------------------')
    print(str(info[1])) # jointName
    print('  index:', info[0])
    print('  damping:', info[6])
    print('  friction:', info[7])
    print('  lower limit:', info[8])
    print('  upper limit:', info[9])
    print('  max force:', info[10])
    print('  max velocity:', info[11])
    print('  link name:', info[12])

numLinks = p.getLinkStates
    
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

positionGains = np.full((numJoints,), 20.0)
velocityGains = np.full((numJoints,), 0.0)

i = 0
while True:
    targetPositions = np.full((numJoints,), np.sin(i / 100.0) * 1.5)
    indices = list(range(numJoints))
    pGains = np.full((numJoints,), 0.0001)
    p.setJointMotorControlArray(op3Id, indices, p.POSITION_CONTROL, targetPositions=targetPositions, positionGains=pGains)
    for _ in range(5):
        p.stepSimulation()
    jointStates = p.getJointStates(op3Id, list(range(numJoints)))
    jointPositions = [x[0] for x in jointStates]
    jointVelocities = [x[1] for x in jointStates]
    contacts = p.getContactPoints(op3Id)
    i += 1
    if i % 200 == 0:
        print(int(i/200), 's')
