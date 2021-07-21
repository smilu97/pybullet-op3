# author: smilu97

import gym
import xacro
import random
import os
import pathlib
import hashlib

from gym import error, spaces, utils
from gym.utils import seeding

from .urdf_env import URDFEnv

class XacroURDFEnv(URDFEnv):
    def __init__(self, filepath, **kwargs):
        doc = xacro.process_file(filepath)
        urdf_content = doc.toprettyxml(indent='  ')
        
        m = hashlib.sha256()
        m.update(bytes(urdf_content, 'utf-8'))
        h = m.digest()[-3:]
        content_hash = (h[0] << 16) | (h[1] << 8) | h[2]

        current_dir = pathlib.Path(__file__).resolve().parent
        tmp_filename = 'xacro_urdf_output_{}.urdf.tmp'.format(content_hash)
        tmp_filepath = os.path.join(current_dir, tmp_filename)

        if os.path.exists(tmp_filepath):
            os.unlink(tmp_filepath)

        fd = open(tmp_filepath, 'w')
        fd.write(urdf_content)
        fd.close()

        super().__init__(tmp_filepath, **kwargs)
