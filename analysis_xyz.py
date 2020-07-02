import numpy as np
import re

a = open("test1").read()
v = re.findall(r"\[(.*)\]", a)
data = np.array([i.split() for i in v], dtype=np.float)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


newd = data[:, 6:9]
newd[:, 1] -= 0.03
newd[:, 2] += 0.01


ax.scatter(data[:, 0], data[:, 1], data[:, 2], color='b')
ax.scatter(newd[:, 0], newd[:, 1], newd[:, 2], color='r')
for i in range(len(data)):
    print(data[i, 0], newd[i, 0])
    ax.plot([data[i, 0], newd[i, 0]],
            [data[i, 1], newd[i, 1]],
            [data[i, 2], newd[i, 2]], 'c')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.show()


