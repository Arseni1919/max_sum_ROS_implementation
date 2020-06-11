#!/usr/bin/env python
# Import random for random numbers
# ------------------------------------ for PyCharm / for ROS
# from scripts.pure_functions import *
from pure_functions import *

# ------------------------------------


# Define the enemy object by extending pygame.sprite.Sprite
# The surface you draw on the screen is now an attribute of 'enemy'
class Target:
    def __init__(self, cell_size=None, order=-1, req=1, surf_center=-1, name=''):
        # super(Target, self).__init__()
        self.cell_size = cell_size
        self.pos = surf_center
        self.req = req
        self.temp_req = req
        self.curr_nei = []
        self.inbox = {}
        self.named_inbox = {}
        self.tuple_keys_inbox = {}
        self.robot_nei_tuples = []
        self.name = name

        if order == -1:
            print('[ERROR]: order of Target == -1')
        self.num = order
        self.name = 'target%s' % order

        self._lock = threading.RLock()

    def get_num_of_agent(self):
        return self.num

    def get_curr_nei(self):
        return self.curr_nei

    def get_access_to_inbox_TAC(self, type_of_requirement, name_of_agent=None, message=None, index_of_iteration=0):
        with self._lock:

            if type_of_requirement == copy_types.copy:
                return copy.deepcopy(self.tuple_keys_inbox)

            if type_of_requirement in message_types:
                self.tuple_keys_inbox[index_of_iteration][(name_of_agent, type_of_requirement)] = message

    def get_pos(self):
        return self.pos

    def get_req(self):
        return self.req

    def get_name(self):
        return self.name
