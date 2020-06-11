#!/usr/bin/env python
# import pygame
import sys
import random
import logging
import threading
import time
# import concurrent.futures
import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math
import numpy as np
import pickle
import json
import copy
from scipy.stats import t
from scipy import stats
import itertools
# from tqdm import tqdm
from pprint import pprint
# import statistics
from collections import namedtuple

CellTuple = namedtuple('CellTuple', ['pos', ])
TargetTuple = namedtuple('TargetTuple', ['pos', 'req', 'name', 'num'])
RobotTuple = namedtuple('AgentTuple',
                        ['pos', 'num_of_robot_nei', 'num_of_target_nei', 'name', 'num', 'cred', 'SR', 'MR'])
MessageType = namedtuple('MessageType', ['from_var_to_func',
                                         'from_var_to_func_only_pos',
                                         'from_var_to_func_dir',
                                         'from_func_pos_collisions_to_var',
                                         'from_func_dir_collisions_to_var',
                                         'from_func_target_to_var'])
message_types = MessageType(from_var_to_func='from_var_to_func',
                            from_var_to_func_only_pos='from_var_to_func_only_pos',
                            from_var_to_func_dir='from_var_to_func_dir',
                            from_func_pos_collisions_to_var='from_func_pos_collisions_to_var',
                            from_func_dir_collisions_to_var='from_func_dir_collisions_to_var',
                            from_func_target_to_var='from_func_target_to_var')
from_func_to_var_types = (message_types.from_func_pos_collisions_to_var, message_types.from_func_target_to_var,
                          message_types.from_func_dir_collisions_to_var)
dictionary_message_types = (message_types.from_func_pos_collisions_to_var, message_types.from_func_target_to_var,
                            message_types.from_func_dir_collisions_to_var, message_types.from_var_to_func,
                            message_types.from_var_to_func_dir)
TypesOfRequirement = namedtuple('TypesOfRequirement', ['copy', 'copy_var_dicts', 'copy_func_dicts'])
copy_types = TypesOfRequirement('copy', 'copy_var_dicts', 'copy_func_dicts')

# OBJECTS = {}


# Define constants for the screen width and height
# SCREEN_WIDTH = 1000
# SCREEN_HEIGHT = 690
# SCREEN_HEIGHT = 850

# have to be odd number for move method of Agent
# CELL_SIZE = {
#     'SMALL': 18,
#     'MEDIUM': 34,
#     'BIG': 74,
#     'CUSTOM': 10,
# }

# SKY_COLOR = (135, 206, 250)
# SPEED_MOVING = 10

# Import pygame.locals for easier access to key coordinates
# Updated to conform to flake8 and black standards
# from pygame.locals import (
#     RLEACCEL,
#     K_UP,
#     K_DOWN,
#     K_LEFT,
#     K_RIGHT,
#     K_ESCAPE,
#     KEYDOWN,
#     QUIT,
# )

# for logging
_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=_format, level=logging.INFO,
                    datefmt="%H:%M:%S")
# logging.getLogger().setLevel(logging.DEBUG)

# -------------------------------------------------------- FOR EXPERIMENT
ITERATIONS = 3
MINI_ITERATIONS = 5
NEED_TO_SAVE_RESULTS = False
POS_POLICY = 'random_furthest'
req = 100
target1 = TargetTuple(pos=(3, 3), req=req, name='target1', num=1)
target2 = TargetTuple(pos=(6, 9), req=req, name='target2', num=2)
target3 = TargetTuple(pos=(0, 5), req=req, name='target3', num=3)
target4 = TargetTuple(pos=(5, 5), req=req, name='target4', num=4)
TARGETS = [target1, target2, target3, target4]
# TARGETS = [target1, target2]

cred = 30
SR = 2.5
MR = 2.5
robot1 = RobotTuple(pos=(1, 7), num_of_robot_nei=None, num_of_target_nei=None, name='robot1', num=1, cred=cred,
                    SR=SR, MR=MR)
robot2 = RobotTuple(pos=(6, 4), num_of_robot_nei=None, num_of_target_nei=None, name='robot2', num=2, cred=cred,
                    SR=SR, MR=MR)
robot3 = RobotTuple(pos=(3, 9), num_of_robot_nei=None, num_of_target_nei=None, name='robot3', num=3, cred=cred,
                    SR=SR, MR=MR)
robot4 = RobotTuple(pos=(4, 5), num_of_robot_nei=None, num_of_target_nei=None, name='robot4', num=4, cred=cred,
                    SR=SR, MR=MR)
ROBOTS = [robot1, robot2, robot3, robot4]
# ROBOTS = [robot1, robot2]

CELLS = []
rows = 10
columns = 10
for r in range(rows):
    for c in range(columns):
        CELLS.append(CellTuple(pos=(r, c)))
# -----------------------------------------------------------------------
