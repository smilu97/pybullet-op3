# author: smilu97

import gym
import xacro
import pybullet as p
import pybullet_data
import numpy as np

from gym import error, spaces, utils
from gym.utils import seeding

class URDFEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(
        self,
        urdf_path,
        gravity=9.81,
        fixed_time_step=0.001,
        num_solver_iterations=100,
        num_steps_per_step=5,
        num_sub_steps=5,
        contact_erp=0.9,
        disable_gui=True,
        start_z=1.0,
        urdf_flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_IMPLICIT_CYLINDER,
        p_gains=None,
        v_gains=None
    ):

        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

        if disable_gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0, 0, -gravity)
        p.setDefaultContactERP(contact_erp)
        p.setPhysicsEngineParameter(
            fixedTimeStep=fixed_time_step,
            numSolverIterations=num_solver_iterations,
            numSubSteps=num_sub_steps
        )

        plane_id = p.loadURDF("plane.urdf")

        print('urdf path:', urdf_path)

        start_pos = [0, 0, start_z]
        start_orientation = p.getQuaternionFromEuler([0,0,0])
        self.robot_id = p.loadURDF(urdf_path, start_pos, start_orientation, flags=urdf_flags)

        self.num_joints = p.getNumJoints(self.robot_id)
        joint_infos = [p.getJointInfo(self.robot_id, i) for i in range(self.num_joints)]
        num_links = p.getLinkStates

        self.p_gains = p_gains or np.full((self.num_joints,), 20.0)
        self.v_gains = v_gains or np.full((self.num_joints,), 0.0)
        self.full_indices = list(range(self.num_joints))
        self.num_steps_per_step = num_steps_per_step

    def step(self, action):
        p.setJointMotorControlArray(
            self.robot_id,
            self.full_indices,
            p.POSITION_CONTROL,
            targetPositions=action,
            positionGains=self.p_gains
        )

        for _ in range(self.num_steps_per_step):
            p.stepSimulation()
        
        joint_states = p.getJointStates(self.robot_id, self.full_indices)
        joint_positions  = [x[0] for x in joint_states]
        joint_velocities = [x[1] for x in joint_states]
    
    def get_contacts():
        return p.getContactPoints(self.robot_id)

    def reset(self):
        pass

    def render(self, mode='human'):
        pass

    def close(self):
        pass
