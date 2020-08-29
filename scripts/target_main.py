#!/usr/bin/env python3
#!/home/hamster/anaconda3/bin/python
# ------------------------------------ for PyCharm / for ROS
# from scripts.Max_sum_FMR_TAC import *
from Max_sum_FMR_TAC import *
# ------------------------------------
import rospy
from std_msgs.msg import Int32, Bool, String


# ------------------------------------ LOCAL VARS
# READY = False
# ------------------------------------


def callback_READY_topic(msg):
    # print(msg.data)
    message = json.loads(msg.data)
    READY_dict[message['iteration']][message['name']] = message['ready']


def callback_CALC_READY_topic(msg):
    message = json.loads(msg.data)
    CALC_READY_dict[message['iteration']][message['name']] = message['ready']


def callback_PREP_rob_tar_topic(msg):
    message = json.loads(msg.data)
    PREP_rob_tar_dict[message['iteration']][message['name']] = {'pos': message['pos'],
                                                                'num_of_robot_nei': message['num_of_robot_nei'],
                                                                'num_of_target_nei': message['num_of_target_nei'],
                                                                'name': message['name'],
                                                                'num': message['num'],
                                                                'cred': message['cred'],
                                                                'SR': message['SR'],
                                                                'MR': message['MR']}


def callback_CALC_topic(msg):
    sender, receiver, message_to_nei, type_of_requirement, index_of_iteration = unpack_json_message(msg.data)
    # print('---')
    # print('here message to %s from %s' % (sender, receiver))
    if receiver == target_object.name:
        # print('---')
        # print('(inside) here message to %s from %s' % (receiver, sender))
        target_object.get_access_to_inbox_TAC(type_of_requirement, sender, message_to_nei, index_of_iteration)


# def start(is_ready):
#     is_ready = True


def wait(curr_iteration):
    message = json.dumps({'name': target_object.name, 'iteration': curr_iteration, 'ready': True})
    # print(message)
    everybody_ready = False
    while not everybody_ready:
        # print(READY_dict)
        pub.publish(message)
        everybody_ready = True
        for target in TARGETS:
            if target.name not in READY_dict[curr_iteration] or not READY_dict[curr_iteration][target.name]:
                everybody_ready = False
                break
        for robot in ROBOTS:
            if robot.name not in READY_dict[curr_iteration] or not READY_dict[curr_iteration][robot.name]:
                everybody_ready = False
                break
        rate.sleep()
    print('[WAIT] - finished')


def calc_wait(curr_iteration):
    message = json.dumps({'name': target_object.name, 'iteration': curr_iteration, 'ready': True})
    # print(message)
    everybody_ready = False
    while not everybody_ready:
        # print(READY_dict)
        pub_CALC_READY_topic.publish(message)
        everybody_ready = True
        for target in TARGETS:
            if target.name not in CALC_READY_dict[curr_iteration]:
                everybody_ready = False
                break
        for robot in ROBOTS:
            if robot.name not in CALC_READY_dict[curr_iteration]:
                everybody_ready = False
                break
        rate.sleep()
    print('[CALC WAIT] - finished')


def first_nei_update_of_robot(curr_iteration):
    target_object.robot_nei_tuples = []
    target_object.tuple_keys_inbox = {}
    for robot in ROBOTS:
        message_dict = PREP_rob_tar_dict[curr_iteration][robot.name]

        # print('--- distance %s ---' % message_dict['name'])
        # print(distance(PREP_rob_tar_dict[curr_iteration][robot.name]['pos'], target_object.pos))
        # print((message_dict['SR'] + message_dict['MR']))

        if distance(PREP_rob_tar_dict[curr_iteration][robot.name]['pos'], target_object.pos) < (message_dict['SR'] +
                                                                                                message_dict['MR']):
            target_object.robot_nei_tuples.append(RobotTuple(pos=tuple(message_dict['pos']),
                                                             num_of_robot_nei=message_dict['num_of_robot_nei'],
                                                             num_of_target_nei=message_dict['num_of_target_nei'],
                                                             name=message_dict['name'],
                                                             num=message_dict['num'],
                                                             cred=message_dict['cred'],
                                                             SR=message_dict['SR'],
                                                             MR=message_dict['MR']))

    for i in range(MINI_ITERATIONS):
        target_object.tuple_keys_inbox[i] = {}


def prep(curr_iteration):
    everybody_sent = False
    while not everybody_sent:
        # print(READY_dict)
        everybody_sent = True
        for robot in ROBOTS:
            if robot.name not in PREP_rob_tar_dict[curr_iteration]:
                everybody_sent = False
                break
        rate.sleep()
    print('[PREP] - finished rob-tar message exchange')

    first_nei_update_of_robot(curr_iteration)
    print('[PREP] - finished final nei update')

    # for robot in target_object.robot_nei_tuples:
    #     print('robot-neighbour: %s' % robot.name)


def calc():

    kwargs = {'agent': target_object, 'for_alg': {
        'mini_iterations': MINI_ITERATIONS,
        'SR': SR,
        'cred': cred,
        'pos_policy': POS_POLICY,
        'pub_CALC_topic': pub_CALC_topic,
    }}
    Max_sum_TAC(kwargs)
    print('[CALC] - finished calc')


def finish():
    # save results
    if NEED_TO_SAVE_RESULTS:
        # if target_object.num == 1:
        list_of_remained_coverages = []
        for curr_iteration in range(ITERATIONS):
            self_req = target_object.req
            for robot in ROBOTS:
                message_dict = PREP_rob_tar_dict[curr_iteration][robot.name]

                # print('--- distance %s ---' % message_dict['name'])
                # print(distance(PREP_rob_tar_dict[curr_iteration][robot.name]['pos'], target_object.pos))
                # print((message_dict['SR'] + message_dict['MR']))

                if distance(message_dict['pos'], target_object.pos) < (message_dict['SR'] + message_dict['MR']):
                    self_req = max(self_req - message_dict['cred'], 0)
            list_of_remained_coverages.append(self_req)
        plt.plot(list_of_remained_coverages)
        plt.show()
        print('[FIN] - finished saving results')
    else:
        print('[FIN] - finished without saving results')


def get_named_tuple_of_target(curr_num_of_target):
    for target in TARGETS:
        # print(type(target.num))
        # print(type(curr_num_of_target))
        if target.num == curr_num_of_target:
            return target
    print('[ERROR]! no named_tuple_of_target')


def create_empty_by_iteration_dict():
    curr_dict = {}
    for i in range(ITERATIONS):
        curr_dict[i] = {}
    return curr_dict


if __name__ == '__main__':
    # ------------------------ INPUT ------------------------ #
    num_of_target = sys.argv[1]
    print('######################### target%s #########################' % num_of_target)
    named_tuple_of_this_target = get_named_tuple_of_target(int(num_of_target))
    target_object = Target(cell_size=1, order=named_tuple_of_this_target.num, req=named_tuple_of_this_target.req,
                           surf_center=named_tuple_of_this_target.pos, name=named_tuple_of_this_target.name)
    READY_dict = create_empty_by_iteration_dict()
    # PREP_rob_rob_dict = create_empty_by_iteration_dict()
    PREP_rob_tar_dict = create_empty_by_iteration_dict()
    CALC_READY_dict = create_empty_by_iteration_dict()
    # ------------------------------------------------------- #
    rospy.init_node('target%s' % sys.argv[1])
    pub = rospy.Publisher('READY_topic', String, latch=True, queue_size=10)
    sub = rospy.Subscriber('READY_topic', String, callback_READY_topic)
    # pub_PREP_rob_rob_topic = rospy.Publisher('PREP_rob_rob_topic', String, latch=True, queue_size=10)
    # sub_PREP_rob_rob_topic = rospy.Subscriber('PREP_rob_rob_topic', String, callback_PREP_rob_rob_topic)
    pub_PREP_rob_tar_topic = rospy.Publisher('PREP_rob_tar_topic', String, latch=True, queue_size=10)
    sub_PREP_rob_tar_topic = rospy.Subscriber('PREP_rob_tar_topic', String, callback_PREP_rob_tar_topic)
    pub_CALC_READY_topic = rospy.Publisher('CALC_READY_topic', String, latch=True, queue_size=50)
    sub_CALC_READY_topic = rospy.Subscriber('CALC_READY_topic', String, callback_CALC_READY_topic)
    pub_CALC_topic = rospy.Publisher('CALC_topic', String, latch=True, queue_size=50)
    sub_CALC_topic = rospy.Subscriber('CALC_topic', String, callback_CALC_topic)
    # pub_FINISH_topic = rospy.Publisher('FINISH_topic', String, latch=True, queue_size=50)
    rate = rospy.Rate(1)  # 1 second

    # start(READY)
    for iteration in range(ITERATIONS):
        print('# --------------------- iteration: %s --------------------- #' % iteration)
        wait(iteration)
        prep(iteration)
        calc_wait(iteration)
        calc()
    finish()
