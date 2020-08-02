from scripts.CONSTANTS import *

# ----------- INPUT ----------- #
x_p = 600
y_p = 550
# ---
x_m = 0
y_m = 0
# ----------- INPUT ----------- #

res = 0.05
origin_x = -25.0
origin_y = -25.0

print('x_m = %s' % (res * x_p + origin_x))
print('y_m = %s' % (res * y_p + origin_y))

print('x_p = %s' % ((x_m - origin_x)/res))
print('y_p = %s' % ((y_m - origin_y)/res))

m = np.array([[2, 2.5], [4, -1]])
b = np.array([0.2, 0])
v = np.array([1/2, 1/2])
print(m)
print()
a = np.asarray(np.dot(m, v) + b)
print(a[0])
print(a[1])
