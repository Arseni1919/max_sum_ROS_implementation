#!/usr/bin/env python
# ------------------------------------ for PyCharm / for ROS
# from scripts.CONSTANTS import *
from CONSTANTS import *
# ------------------------------------



def distance(pos1, pos2):
    """
    input:
    output:
    """
    return math.sqrt(math.pow(pos1[0] - pos2[0], 2) + math.pow(pos1[1] - pos2[1], 2))


def in_area(pos_1, pos_2, SR):
    """
    input:
    output:
    """
    px, py = pos_1
    tx, ty = pos_2
    return math.sqrt(math.pow(px - tx, 2) + math.pow(py - ty, 2)) < SR


def foo():
    print('here')
    logging.info("Thread %s : starting foo", threading.get_ident())
    time.sleep(0.1)
    logging.info("Thread %s : finishing foo", threading.get_ident())


def getAngle(a, b, c):
    ang = math.degrees(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))
    curr_ang = ang + 360 if ang < 0 else ang
    small_ang = 360 - curr_ang if curr_ang > 180 else curr_ang
    return small_ang