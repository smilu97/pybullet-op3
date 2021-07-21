# author: smilu97

import gym
import xacro
import random
import os
import pathlib
from gym import error, spaces, utils
from gym.utils import seeding

from .urdf_env import URDFEnv

class XacroURDFEnv(URDFEnv):
    def __init__(self, filepath, **kwargs):
        doc = xacro.process_file(filepath)
        urdf_content = doc.toprettyxml(indent='  ')
        
        random_index = random.randint(1000000, 9999999)
        current_dir = pathlib.Path(__file__).resolve().parent
        tmp_filename = 'xacro_urdf_output_{}.urdf.tmp'.format(random_index)
        tmp_filepath = os.path.join(current_dir, tmp_filename)

        if os.path.exists(tmp_filepath):
            os.unlink(tmp_filepath)

        fd = open(tmp_filepath, 'w')
        fd.write(urdf_content)
        fd.close()

        super().__init__(tmp_filepath, **kwargs)
