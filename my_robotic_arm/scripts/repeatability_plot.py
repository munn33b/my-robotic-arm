#!/usr/bin/env python3

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

data = np.array([
    [0.37460541698078487, 0.37019280969383206, 0.3613983330669459, 0.37309073752372635, 0.3690294673234686],
    [0.4578332262969919, 0.45844440058168545, 0.4637913675857817, 0.45613747078953004, 0.45550538576785676],
    [0.347154275889843, 0.3781647924809857, 0.3754631950668236, 0.37001803885618084, 0.3766035827372678]
])
data_x, data_y, data_z = data
print (data)

ax.scatter(data_x, data_y, data_z, c='blue', s=50, marker='s')

ax.set_xlabel("Position in X Axis")
ax.set_ylabel("Position in Y Axis")
ax.set_zlabel("Position in Z Axis")
plt.show()
