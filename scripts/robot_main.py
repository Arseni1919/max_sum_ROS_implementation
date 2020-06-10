#!/usr/bin/env python

# ------------------------------------ for PyCharm
from scripts.Max_sum_FMR_TAC import *
# ------------------------------------ for ROS
# from Max_sum_FMR_TAC import *
# ------------------------------------
import rospy
from std_msgs.msg import Int32, Bool, String
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from nav_msgs.msg import Odometry
# from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *


# ------------------------------------ LOCAL VARS
# READY = False
# ------------------------------------


def callback_READY_topic(msg):
    # print(msg.data)
    message = json.loads(msg.data)
    READY_dict[message['iteration']][message['name']] = message['ready']


def callback_PREP_rob_rob_topic(msg):
    message = json.loads(msg.data)
    PREP_rob_rob_dict[message['iteration']][message['name']] = message['pos']


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


def callback_amcl(msg):
    print('---')
    print('x_pose_amcl: %s' % msg.pose.pose.position.x)
    print('y_pose_amcl: %s' % msg.pose.pose.position.y)
    print('---')
    robot_object.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    rate.sleep()


# def start(is_ready):
#     is_ready = True


def wait(curr_iteration):
    message = json.dumps({'name': robot_object.get_name(), 'iteration': curr_iteration, 'ready': True})
    # print(message)
    everybody_ready = False
    while not everybody_ready:
        # print(READY_dict)
        pub_READY_topic.publish(message)
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


def first_nei_update_of_robot(curr_iteration):
    robot_object.target_nei_tuples = []
    robot_object.robot_nei_tuples = []
    robot_object.all_nei_tuples = []
    robot_object.tuple_keys_inbox = {}
    for target in TARGETS:
        if distance(target.pos, robot_object.pos) < (robot_object.SR + robot_object.MR):
            robot_object.target_nei_tuples.append(target)
    for robot in ROBOTS:
        if distance(PREP_rob_rob_dict[curr_iteration][robot.name], robot_object.pos) < (robot.MR + robot_object.MR):
            robot_object.robot_nei_tuples.append(robot)
    robot_object.all_nei_tuples.extend(robot_object.target_nei_tuples)

    for i in range(MINI_ITERATIONS):
        robot_object.tuple_keys_inbox[i] = {}

    robot_object.future_pos = robot_object.pos


def final_nei_update_of_robot(curr_iteration):
    new_robot_nei_tuples = []
    for robot in robot_object.robot_nei_tuples:
        message_dict = PREP_rob_tar_dict[curr_iteration][robot.name]
        new_robot_nei_tuples.append(RobotTuple(pos=message_dict['pos'],
                                               num_of_robot_nei=message_dict['num_of_robot_nei'],
                                               num_of_target_nei=message_dict['num_of_target_nei'],
                                               name=message_dict['name'],
                                               num=message_dict['num'],
                                               cred=message_dict['cred'],
                                               SR=message_dict['SR'],
                                               MR=message_dict['MR']))

    robot_object.robot_nei_tuples = new_robot_nei_tuples
    robot_object.all_nei_tuples.extend(robot_object.robot_nei_tuples)


def prep(curr_iteration):
    message = json.dumps({'name': robot_object.name, 'iteration': curr_iteration, 'pos': robot_object.pos})
    everybody_sent = False
    while not everybody_sent:
        # print(READY_dict)
        pub_PREP_rob_rob_topic.publish(message)
        everybody_sent = True
        for robot in ROBOTS:
            if robot.name not in PREP_rob_rob_dict[curr_iteration]:
                everybody_sent = False
                break
        rate.sleep()
    print('--- PREP_rob_rob_dict: ---')
    print(PREP_rob_rob_dict)
    print('[PREP] - finished rob-rob message exchange')

    first_nei_update_of_robot(curr_iteration)

    message = json.dumps({'name': robot_object.name, 'iteration': curr_iteration, 'pos': robot_object.pos,
                          'cred': robot_object.cred, 'num': robot_object.num,
                          'num_of_target_nei': len(robot_object.target_nei_tuples),
                          'num_of_robot_nei': len(robot_object.robot_nei_tuples),
                          'SR': robot_object.SR, 'MR': robot_object.MR})
    everybody_sent = False
    while not everybody_sent:
        # print(READY_dict)
        pub_PREP_rob_tar_topic.publish(message)
        everybody_sent = True
        for robot in ROBOTS:
            if robot.name not in PREP_rob_tar_dict[curr_iteration]:
                everybody_sent = False
                break
        rate.sleep()
    print('--- PREP_rob_tar_dict: ---')
    print(PREP_rob_tar_dict)
    print('[PREP] - finished rob-tar message exchange')

    final_nei_update_of_robot(curr_iteration)

    print('--- robot_object.robot_nei_tuples: ---')
    for robot in robot_object.robot_nei_tuples:
        print('robot-neighbour: %s' % robot.name)
    print('--- robot_object.target_nei_tuples: ---')
    for target in robot_object.target_nei_tuples:
        print('target-neighbour: %s' % target.name)


def calc():
    kwargs = {'agent': robot_object, 'for_alg': {
        'mini_iterations': MINI_ITERATIONS,
        'SR': SR,
        'cred': cred,
        'pos_policy': POS_POLICY,
    }}
    return Max_sum_TAC(kwargs)


def move(curr_iteration, to_pos):
    if curr_iteration != 0:
        # maybe we have to wait to others here
        pass
    # client.send_goal(goal_pose(to_pos))
    # client.wait_for_result()
    pass


def finish():
    pass


def get_named_tuple_of_robot(curr_num_of_robot):
    for robot in ROBOTS:
        # print(type(target.num))
        # print(type(curr_num_of_target))
        if robot.num == curr_num_of_robot:
            return robot
    print('[ERROR]! no named_tuple_of_target')


def create_empty_by_iteration_dict():
    curr_dict = {}
    for i in range(ITERATIONS):
        curr_dict[i] = {}
    return curr_dict


def goal_pose(pose):
    # print("in goal_pose %s" % (datetime.datetime.now()))
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = float(pose[0])
    goal_pose.target_pose.pose.position.y = float(pose[1])
    goal_pose.target_pose.pose.position.z = 0.0
    goal_pose.target_pose.pose.orientation.x = 0.0
    goal_pose.target_pose.pose.orientation.y = 0.0
    goal_pose.target_pose.pose.orientation.z = 0.0
    goal_pose.target_pose.pose.orientation.w = 1.0

    return goal_pose


if __name__ == '__main__':
    # ------------------------ INPUT ------------------------ #
    num_of_robot = sys.argv[1]
    print('######################### robot%s #########################' % num_of_robot)
    named_tuple_of_this_robot = get_named_tuple_of_robot(int(num_of_robot))
    robot_object = Robot(number_of_robot=named_tuple_of_this_robot.num,
                         cell_size=1, surf_center=named_tuple_of_this_robot.pos,
                         MR=named_tuple_of_this_robot.MR, SR=named_tuple_of_this_robot.SR,
                         cred=named_tuple_of_this_robot.cred, name=named_tuple_of_this_robot.name)
    READY_dict = create_empty_by_iteration_dict()
    PREP_rob_rob_dict = create_empty_by_iteration_dict()
    PREP_rob_tar_dict = create_empty_by_iteration_dict()
    # ------------------------------------------------------- #
    rospy.init_node('robot%s' % sys.argv[1])
    pub_READY_topic = rospy.Publisher('READY_topic', String, latch=True, queue_size=10)
    sub_READY_topic = rospy.Subscriber('READY_topic', String, callback_READY_topic)
    pub_PREP_rob_rob_topic = rospy.Publisher('PREP_rob_rob_topic', String, latch=True, queue_size=10)
    sub_PREP_rob_rob_topic = rospy.Subscriber('PREP_rob_rob_topic', String, callback_PREP_rob_rob_topic)
    pub_PREP_rob_tar_topic = rospy.Publisher('PREP_rob_tar_topic', String, latch=True, queue_size=10)
    sub_PREP_rob_tar_topic = rospy.Subscriber('PREP_rob_tar_topic', String, callback_PREP_rob_tar_topic)
    # sub_amcl = rospy.Subscriber('/agent%s/amcl_pose' % sys.argv[1], PoseWithCovarianceStamped, callback_amcl)
    rate = rospy.Rate(1)  # 1 second

    # client = actionlib.SimpleActionClient('agent%s/move_base' % sys.argv[1], MoveBaseAction)
    # client.wait_for_server()

    # client.send_goal(goal_pose(named_tuple_of_this_robot.pos))
    # client.wait_for_result()
    # move(0, named_tuple_of_this_robot.pos)

    # start(READY)
    for iteration in range(ITERATIONS):
        print('# --------------------- iteration: %s --------------------- #' % iteration)
        wait(iteration)
        prep(iteration)
        next_pos = calc()
        move(iteration, next_pos)
    finish()
