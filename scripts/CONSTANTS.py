#!/usr/bin/env python
# import pygame
from __future__ import print_function
from decimal import Decimal
from prettytable import PrettyTable
import sys
# print(sys.version)
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

# for logging
_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=_format, level=logging.INFO,
                    datefmt="%H:%M:%S")
# logging.getLogger().setLevel(logging.DEBUG)

# -------------------------------------------------------- FOR EXPERIMENT
ITERATIONS = 3
MINI_ITERATIONS = 5
NEED_TO_SAVE_RESULTS = True
POS_POLICY = 'random_furthest'
req = 100
target1 = TargetTuple(pos=(1, -1), req=req, name='target1', num=1)
target2 = TargetTuple(pos=(2, -1), req=req, name='target2', num=2)
target3 = TargetTuple(pos=(3, -2), req=req, name='target3', num=3)
target4 = TargetTuple(pos=(4, -2), req=req, name='target4', num=4)
# TARGETS = [target1, target2, target3, target4]
TARGETS = [target1, target3]

cred = 30
SR = 1.0
MR = 2.5
start_pose1 = (2, -1)
start_pose2 = (2, 0)
start_pose3 = (0.5, 0.5)
start_pose4 = (1.5, 1.5)

start_pose5 = (2.1, 1)
# start_pose6 = (2.0, -3.1)
start_pose6 = (0.0, 0.0)

robot1 = RobotTuple(pos=start_pose1, num_of_robot_nei=None, num_of_target_nei=None, name='robot1', num=1, cred=cred,
                    SR=SR, MR=MR)
# robot2 = RobotTuple(pos=(2.1, 3), num_of_robot_nei=None, num_of_target_nei=None, name='robot2', num=2, cred=cred,
#                     SR=SR, MR=MR)
# robot5 = RobotTuple(pos=(2.1, 3), num_of_robot_nei=None, num_of_target_nei=None, name='robot5', num=5, cred=cred,
#                     SR=SR, MR=MR)
robot3 = RobotTuple(pos=start_pose2, num_of_robot_nei=None, num_of_target_nei=None, name='robot3', num=3, cred=cred,
                    SR=SR, MR=MR)
# robot4 = RobotTuple(pos=(1.2, 1), num_of_robot_nei=None, num_of_target_nei=None, name='robot4', num=4, cred=cred,
#                     SR=SR, MR=MR)

robot4 = RobotTuple(pos=start_pose4, num_of_robot_nei=None, num_of_target_nei=None, name='robot4', num=4, cred=cred,
                    SR=SR, MR=MR)
robot5 = RobotTuple(pos=start_pose5, num_of_robot_nei=None, num_of_target_nei=None, name='robot5', num=5, cred=cred,
                    SR=SR, MR=MR)

# ROBOTS = [robot1, robot3, robot4, robot5]
# ROBOTS = [robot1, robot2]
ROBOTS = [robot3, robot5]

CELLS = []
rows = 10
columns = 10
# Transformation
m = np.array([[2.0, 2.1], [-3.1, 1]])
b = np.array([0, 0])
# m = np.array([[4, -1], [2, 2.5]])
# b = np.array([0, 0.2])
field = PrettyTable()

field.field_names = [i+1 for i in range(rows)]
for r in range(rows):
    # print()
    raw = []
    for c in range(columns):
        v = np.array([r/float(rows), c/float(columns)])
        out = np.asarray(np.dot(m, v) + b)
        # print(sys.version)
        raw.append('%s,%s'.expandtabs(10) % (round(Decimal(out[0]),1), round(Decimal(out[1]),1)))
        # print('%s,%s \t'.expandtabs(10) % (round(Decimal(out[0]),1), round(Decimal(out[1]),1)), end='')
        CELLS.append(CellTuple(pos=(out[0], out[1])))
    field.add_row(raw)
print(field)
#
# -----------------------------------------------------------------------
