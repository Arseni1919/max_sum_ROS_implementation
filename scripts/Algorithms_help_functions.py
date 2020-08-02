#!/usr/bin/env python

# ------------------------------------ for PyCharm / for ROS
# from scripts.pure_functions import *
from pure_functions import *
# ------------------------------------

'''
OUTER WORLD - ALL FUNCTIONS HERE SUPPOSE TO BE TRANSPARENT TO SIMULATION
'''


# ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------
# ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------
# help Algorithms functions
# ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------
# ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------
def foo():
    """
    input:
    output:
    """
    pass


def get_possible_pos_with_MR_general(self_agent):
    possible_pos = []
    help_set = []

    # all cells that are in MR range
    for cell in self_agent.cells:
        # print(cell.pos)
        if distance(self_agent.get_pos(), cell.pos) < self_agent.get_MR():
            help_set.append(cell)

    cell_set = help_set
    help_set = []

    # minus agents' cells
    for cell in cell_set:
        captured = False
        for agent in self_agent.robot_nei_tuples:
            # if self_agent.num == 4 and agent.num == 3:
            #     print('%s against %s is %s' % (agent.pos, cell.pos, (agent.pos == cell.pos)))
            if agent.pos == cell.pos and agent.num != self_agent.get_num_of_agent():
                captured = True
                break
        if not captured:
            help_set.append(cell)

    cell_set = help_set
    help_set = []

    # minus targets' cells
    for cell in cell_set:
        captured = False
        for target in self_agent.target_nei_tuples:
            if target.pos == cell.pos:
                captured = True
                break
        if not captured:
            help_set.append(cell)

    cell_set = help_set
    help_set = []

    # copy only the positions
    for cell in cell_set:
        possible_pos.append(cell.pos)
    return possible_pos


# ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------
# For Max_sum:
# ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------

def max_sum_create_null_variable_message(possible_pos):
    message = {}
    for pos in possible_pos:
        message[pos] = 0
    return message


def new_function_message(possible_pos, target, SR, tuple_for_new_message):
    new_message = max_sum_create_null_variable_message(possible_pos)
    for pos in possible_pos:
        if distance(pos, target.get_pos()) < SR:
            new_message[pos] = tuple_for_new_message[0]
        else:
            new_message[pos] = tuple_for_new_message[1]

    return new_message


def get_SR_new_message(max_values, target_req, target_num, cred, message_to):
    '''
    :param max_values: dictionary
    :param target_req: int
    :param cred: int
    :return:
    '''
    tuple_for_new_message = {0: 0, 1: 0}
    neighbours = max_values.keys()
    num_of_neighbours = len(neighbours)

    # if message_to == 1 and target_num == 1:
    #     print('max_values from target ', target_num, 'to robot ', message_to, ': ', max_values)

    # 0 - INSIDE SR | 1 - OUTSIDE of SR
    # go through all combinations
    for comb_tuple in itertools.product(range(2), repeat=num_of_neighbours):

        in_or_out_of_SR_of_message_to = -1

        # check for impossible_combination
        impossible_combination = False
        for curr_nei, in_or_out_of_SR in zip(neighbours, comb_tuple):
            if max_values[curr_nei][in_or_out_of_SR] == -1:
                impossible_combination = True
        if impossible_combination:
            # print('[ERROR]: impossible_combination')
            continue

        # calculating table_column and result_column
        result_column = 0
        # coverage = target_req
        remained_coverage = target_req
        message_to_is_out_of_SR = False
        for curr_nei, in_or_out_of_SR in zip(neighbours, comb_tuple):
            result_column += max_values[curr_nei][in_or_out_of_SR]
            if curr_nei == message_to:
                in_or_out_of_SR_of_message_to = in_or_out_of_SR  # we'll use this later
                if in_or_out_of_SR == 1:
                    remained_coverage = 0
                    message_to_is_out_of_SR = True
            else:
                if not message_to_is_out_of_SR and in_or_out_of_SR == 0:
                    remained_coverage = max(remained_coverage - cred, 0)
        result_column += min(remained_coverage, cred)
        # if message_to == 1: print(result_column)

        # choosing the maximum per each in-SR and out-SR value
        if tuple_for_new_message[in_or_out_of_SR_of_message_to] < result_column:
            tuple_for_new_message[in_or_out_of_SR_of_message_to] = result_column
    # if message_to == 1: print(tuple_for_new_message)
    # minus alpha
    min_value = min(tuple_for_new_message[0], tuple_for_new_message[1])
    tuple_for_new_message[0] = tuple_for_new_message[0] - min_value
    tuple_for_new_message[1] = tuple_for_new_message[1] - min_value

    return tuple_for_new_message


def get_set_of_max_pos(agent, sum_of_all_messages, pos_policy):
    # the max value
    max_value = max(sum_of_all_messages.values())

    # array of positions with maximal value
    set_of_max_pos = []
    for pos, value in sum_of_all_messages.items():
        if value == max_value:
            set_of_max_pos.append(pos)

    if max_value < 0:
        print('[ERROR]: something strange: max_value = -1')

    if max_value == 0:
        if pos_policy == 'random_furthest':
            set_of_furthest_max_pos = get_set_of_furthest_max_pos(agent, set_of_max_pos)
            return set_of_furthest_max_pos
        # if pos_policy == 'random_furthest_directed':
        #     set_of_furthest_directed_max_pos = get_set_of_furthest_directed_max_pos(agent, set_of_max_pos)
        #     return set_of_furthest_directed_max_pos
    return set_of_max_pos


def get_set_of_furthest_max_pos(agent, set_of_max_pos):
    curr_pos = agent.get_pos()
    cell_size = agent.get_cell_size()
    max_dist = 0
    for pos in set_of_max_pos:
        if distance(curr_pos, pos) > max_dist:
            max_dist = distance(curr_pos, pos)

    set_of_furthest_max_pos = []
    for pos in set_of_max_pos:
        if distance(curr_pos, pos) > (max_dist - cell_size):
            set_of_furthest_max_pos.append(pos)
    # print(len(set_of_furthest_max_pos))
    return set_of_furthest_max_pos

# ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------
# ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------
# ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------
# ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------ ------
