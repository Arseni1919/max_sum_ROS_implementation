from __future__ import print_function

import numpy as np
from decimal import Decimal
from prettytable import PrettyTable

# x = 0.8
# y = 0.0
#
# m = np.array([[1.65, 0.28], [0, 1.81]])
# b = np.array([-0.4, 0.69])
# print('m: \n%s' % m)
# print('b: %s' % b)
# v = np.array([x, y])
# print('transformation: %s' % np.asarray(np.dot(m, v) + b))
#
# rows = 5
# columns = 5
# field = PrettyTable()
# field.field_names = [i+1 for i in range(rows)]
#
# for r in range(rows-1, -1, -1):
#     raw = []
#     # print('---')
#     for c in range(columns):
#         v = np.array([c/float(columns-1), r / float(rows-1)])
#         # print([c/float(columns-1), r / float(rows-1)])
#         out = np.asarray(np.dot(m, v) + b)
#         raw.append('%s,%s' % (round(out[0],2), round(out[1],2)))
#         # CELLS.append(CellTuple(pos=(out[0], out[1])))
#     field.add_row(raw)
#
# print('field: \n%s' % field)

rows = 5
columns = 5
row_divisions = []
column_divisions = []
for i in range(rows):
    row_divisions.append(i/float(rows-1))
for i in range(columns):
    column_divisions.append(i/float(columns-1))
print('row_divisions:', row_divisions)
print('column_divisions:', column_divisions)
#################
# p_2 . . . p_3 #
# .         .   #
# .         .   #
# .         .   #
# p_1 . . . p_4 #
#################
p_1 = (0.52, 0.08)
p_2 = (0.65, 2.03)
p_3 = (2.13, 2.04)
p_4 = (2.18, 0.07)

field = PrettyTable()
field.field_names = [i+1 for i in range(columns)]


def get_xy(p1, p2, p3, p4, row_division, column_division):

    ax_left = p1[0] + row_division * (p2[0] - p1[0])
    ax_right = p4[0] + row_division * (p3[0] - p4[0])
    x = ax_left + column_division * (ax_right - ax_left)

    ax_top = p2[1] + column_division * (p3[1] - p2[1])
    ax_bottom = p1[1] + column_division * (p4[1] - p1[1])
    y = ax_bottom + row_division * (ax_top - ax_bottom)

    return x, y


for r in range(rows-1, -1, -1):
    raw = []
    print('---', r)
    for c in range(columns):
        curr_x, curr_y = get_xy(p_1, p_2, p_3, p_4, row_divisions[r], column_divisions[c])
        v = np.array([curr_x, curr_y])
        print(' ', c, v)
        # out = np.asarray(np.dot(m, v) + b)
        raw.append('%s,%s' % (round(v[0], 2), round(v[1], 2)))
        CELLS.append(CellTuple(pos=(v[0], v[1])))
    field.add_row(raw)

print('field: \n%s' % field)