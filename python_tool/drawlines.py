import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from numpy import linalg as LA
# from mpl_toolkits.mplot3d import Axes3D


t0_v = np.array([-0.0369801, 4.9809915, -0.1591650])

t0_g = np.array([4.650712, 1.999425, 0.055847])

t1_v = np.array([-0.1029960, 9.0123400, -0.2488936])

t1_g = np.array([8.354912, 3.548564, 0.035288])

print("v distance: {:.2f}".format(LA.norm(t1_v -t0_v)))
print("g distance: {:.2f}".format(LA.norm(t1_g -t0_g)))

fig = plt.figure()
# ax = fig.gca(projection='3d')

plt.plot([t0_v[0], t1_v[0] ],[t0_v[1], t1_v[1] ],c = "red")

plt.plot([t0_g[0], t1_g[0] ],[t0_g[1], t1_g[1] ],c = "blue")

plt.xlabel('X axis(East)')
plt.ylabel('Y axis(North)')
# plt.gca().set_aspect('equal', adjustable='box')
plt.show()
