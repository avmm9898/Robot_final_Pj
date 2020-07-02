import numpy as np
import re

a = open("test1").read()
v = re.findall(r"\[(.*)\]", a)
data = np.array([i.split() for i in v], dtype=np.float)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(data[:, 0], data[:, 1], data[:, 2], color='b')
ax.scatter(data[:, 6], data[:, 7], data[:, 8], color='r')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.show()


