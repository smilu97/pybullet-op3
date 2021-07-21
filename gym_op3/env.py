# author: smilu97

import gym
import xacro
import os
import pathlib
from gym import error, spaces, utils
from gym.utils import seeding

from .xacro_urdf_env import XacroURDFEnv

class OP3Env(XacroURDFEnv):
    def __init__(self, **kwargs):
        current_dir = pathlib.Path(__file__).resolve().parent
        xacro_path = os.path.join(current_dir, 'urdf/robotis_op3.urdf.xacro')
        super().__init__(xacro_path, **kwargs)
