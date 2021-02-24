import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

"""
Robot angular velocity plot
@author: Mingze
@contact: mingzec@gmail.com
@date: 02.2021
"""
# read from csv and save corresponding column data to variables
df = pd.read_csv('angularVel.csv')
# saved_column = df.column_name  # can also use df['column_name']
av_x = df.av_x
av_y = df.av_y
av_z = df.av_z
t = df.time

fig, axs = plt.subplots(3)
fig.suptitle('Angular velocity along all 3 axes')
axs[0].plot(t, av_x)
axs[0].set_title('Angular velocity along X-axis')
axs[1].plot(t, av_y)
axs[1].set_title('Angular velocity along Y-axis')
axs[2].plot(t, av_z)
axs[2].set_title('Angular velocity along Z-axis')
fig.tight_layout()
plt.show()
fig.savefig('angularVel.png')
