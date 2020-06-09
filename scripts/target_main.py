#!/usr/bin/env python

# ------------------------------------ for PyCharm
# from scripts.CONSTANTS import *
# ------------------------------------ for ROS
from CONSTANTS import *
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



# def start(is_ready):
#     is_ready = True


def wait(curr_iteration, tuple_of_this_target):
    message = json.dumps({'name': tuple_of_this_target.name, 'iteration': curr_iteration, 'ready': True})
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
            # if not READY_dict[curr_iteration][target.name]:
            #     everybody_ready = False
            #     break
        rate.sleep()


def prep():
    pass


def calc():
    pass


def finish():
    pass


def get_named_tuple_of_target(curr_num_of_target):
    for target in TARGETS:
        # print(type(target.num))
        # print(type(curr_num_of_target))
        if target.num == curr_num_of_target:
            return target
    print('[ERROR]! no named_tuple_of_target')


def create_READY_dict():
    curr_dict = {}
    for i in range(ITERATIONS):
        curr_dict[i] = {}
    return curr_dict


if __name__ == '__main__':
    # ------------------------ INPUT ------------------------ #
    num_of_target = sys.argv[1]
    print('######################### target%s #########################' % num_of_target)
    named_tuple_of_this_target = get_named_tuple_of_target(int(num_of_target))
    READY_dict = create_READY_dict()
    # ------------------------------------------------------- #
    rospy.init_node('target%s' % sys.argv[1])
    pub = rospy.Publisher('READY_topic', String, latch=True, queue_size=10)
    sub = rospy.Subscriber('READY_topic', String, callback_READY_topic)
    rate = rospy.Rate(1)  # 1 second

    # start(READY)
    for iteration in range(ITERATIONS):
        print('# --------------------- iteration: %s --------------------- #' % iteration)
        wait(iteration, named_tuple_of_this_target)
        prep()
        calc()
    finish()
