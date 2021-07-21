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
        joint_indices,
        gravity=9.81,
        fixed_time_step=0.001,
        num_solver_iterations=200,
        num_steps_per_step=5,
        num_sub_steps=0,
        contact_erp=0.9,
        disable_gui=True,
        start_z=1.0,
        urdf_flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS,
        p_gains=None,
        v_gains=None
    ):

        self.urdf_flags = urdf_flags
        self.joint_indices = joint_indices
        self.start_z = start_z
        self.gravity = gravity
        self.contact_erp = contact_erp
        self.fixed_time_step = fixed_time_step
        self.num_solver_iterations = num_solver_iterations
        self.num_sub_steps = num_sub_steps
        self.urdf_path = urdf_path

        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

        if disable_gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        
        self._spawn()
        self._setPhysicsParameters()
        
        self.num_joints = p.getNumJoints(self.robot_id)

        n = len(self.joint_indices)
        self.p_gains = p_gains or np.full((n,), 0.0001)
        self.v_gains = v_gains or np.full((n,), 0.0001)        
        self.num_steps_per_step = num_steps_per_step

    def step(self, action):
        p.setJointMotorControlArray(
            self.robot_id,
            self.joint_indices,
            p.POSITION_CONTROL,
            # forces=action,
            targetPositions=action,
            positionGains=self.p_gains,
            velocityGains=self.v_gains
        )

        for _ in range(self.num_steps_per_step):
            p.stepSimulation()
            
        return [], False
    
    def get_contacts(self):
        return p.getContactPoints(self.robot_id)

    def reset(self):
        p.resetSimulation()
        self._spawn()
        self._setPhysicsParameters()

    def _spawn(self):
        p.loadURDF("plane.urdf")

        start_pos = [0, 0, self.start_z]
        start_orientation = p.getQuaternionFromEuler([0,0,0])
        self.robot_id = p.loadURDF(self.urdf_path, start_pos, start_orientation, flags=self.urdf_flags, globalScaling=5.0)
    
    def _setPhysicsParameters(self):
        p.setDefaultContactERP(self.contact_erp)
        p.setGravity(0, 0, -self.gravity)
        p.setPhysicsEngineParameter(
            fixedTimeStep=self.fixed_time_step,
            numSolverIterations=self.num_solver_iterations,
            numSubSteps=self.num_sub_steps
        )

    def render(self, mode='human'):
        pass

    def close(self):
        pass
