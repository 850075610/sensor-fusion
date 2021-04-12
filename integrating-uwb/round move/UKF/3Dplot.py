from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

# read from csv and save corresponding column data to variables
# attention: if there's no enough data, it will be a point rather than a
# trace/trajectory

df = pd.read_csv('pose1204.csv')
# saved_column = df.column_name  # can also use df['column_name']
# x and y are coordinates based on calculation
x = df.x
y = df.y
uwb_x = df.uwb_x
uwb_y = df.uwb_y
z = df.time_in_sec

f = plt.figure(figsize=(15, 8))
ax = f.add_subplot(111, projection='3d')
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
# ax.set_zlabel('t', y=1.08)
ax.set_zlabel('time')
# ax.zaxis.set_label_coords(1.05, 0.05)

real, = ax.plot(x, y, z, label='Real movement')
tag, = ax.plot(uwb_x, uwb_y, z, label='tag')
# ax.legend()
start = ax.scatter(x[0], y[0], z[0], marker="v", c="r")
end = ax.scatter(x.iat[-1], y.iat[-1], z.iat[-1], marker="o", c="g")
# ax.text(x[0], y[0], z[0], "Start", color='red')
ax.legend((start, end, real, tag),
          ("Start", "End", "Pose estimation based on UKF", "Calculated tag"), loc=0)
plt.show()
# plt.savefig('ukf.png', bbox_inches='tight', dpi=100)
