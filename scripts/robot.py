#!/usr/bin/env python
# ------------------------------------ for PyCharm / for ROS
# from scripts.pure_functions import *
from pure_functions import *
# ------------------------------------
import rospy


# Define a player object by extending pygame.sprite.Sprite
# The surface drawn on the screen is now an attribute of 'player'
class Robot:
    def __init__(self,
                 number_of_robot,
                 cell_size=None,
                 surf_center=-1,
                 MR=-1,
                 SR=-1,
                 cred=5,
                 name='',
                 cells=None
                 ):
        # super(Robot, self).__init__()
        self.cell_size = cell_size
        self.pos = surf_center
        self.num = number_of_robot
        self.name = name
        self.MR = MR
        self.SR = SR
        self.cred = cred
        self.future_pos = None
        self.tuple_keys_inbox = {}
        self._lock = threading.RLock()
        self.cells = cells
        self.target_nei_tuples = []
        self.robot_nei_tuples = []
        self.all_nei_tuples = []

    # different from other because the agent is moving -> NOT self.surf_center
    def get_pos(self):
        return self.pos

    def get_cell_size(self):
        return self.cell_size

    def get_access_to_inbox_TAC(self, type_of_requirement, name_of_agent=None, message=None, index_of_iteration=0):

        with self._lock:

            if type_of_requirement == copy_types.copy:
                return copy.deepcopy(self.tuple_keys_inbox)

            if type_of_requirement in message_types:
                self.tuple_keys_inbox[index_of_iteration][(name_of_agent, type_of_requirement)] = message

    def get_SR(self):
        return self.SR

    def get_MR(self):
        return self.MR

    def get_name(self):
        return self.name

    def get_cred(self):
        return self.cred

    def get_num_of_agent(self):
        return self.num




