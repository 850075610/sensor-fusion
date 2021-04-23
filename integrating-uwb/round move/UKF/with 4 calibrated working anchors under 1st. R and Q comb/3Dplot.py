#!/usr/bin/env python
"""3Dplot.py: Plot coordinates of the robot with different colors for each round."""
__author__ = "Mingze Chen"
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

# read from csv and save corresponding column data to variables
# attention: if there's no enough data, it will be a point rather than a
# trace/trajectory

df = pd.read_csv('pose2204.csv')
# saved_column = df.column_name  # can also use df['column_name']
# x and y are coordinates based on calculation
x = df.x
# pandas.core.series.Series
# print(f"type(x) = {type(x)}")
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
# 1st. round
real, = ax.plot(x[:355], y[:355], z[:355], label='Real movement')
tag, = ax.plot(uwb_x[:355], uwb_y[:355], z[:355], label='tag')

# 2nd. round
real, = ax.plot(x[356:647], y[356:647], z[356:647], label='Real movement')
tag, = ax.plot(uwb_x[356:647], uwb_y[356:647], z[356:647], label='tag')

# 3rd. round
real, = ax.plot(x[648:954], y[648:954], z[648:954],
                label='Real movement', c='white')
tag, = ax.plot(uwb_x[648:954], uwb_y[648:954], z[648:954], label='tag')

# 4th round
real, = ax.plot(x[955:], y[955:], z[955:], label='Real movement', c='blue')
tag, = ax.plot(uwb_x[955:], uwb_y[955:], z[955:], label='tag')
# ax.legend()
start = ax.scatter(x[0], y[0], z[0], marker="v", c="r")
end = ax.scatter(x.iat[-1], y.iat[-1], z.iat[-1], marker="o", c="g")
# ax.text(x[0], y[0], z[0], "Start", color='red')
ax.legend((start, end),
          ("Start", "End"), loc=0)
# plt.show()
plt.savefig('ukf_cali_2204.png', bbox_inches='tight', dpi=100)
