import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

"""
Kalman filter plot
@author: Mingze
@contact: mingzec@gmail.com
@date: 05.2021
"""
# read from csv and save corresponding column data to variables
df = pd.read_csv('kf3105.csv')
# saved_column = df.column_name  # can also use df['column_name']
pre_x = df.predicted_x
corrected_x = df.corrected_x
uwb_x = df.uwb_x
t = df.time


# fig, axs = plt.subplots(3)
# fig.suptitle('Kalman Filter Result for 1m movement')
# axs[0].plot(t, pre_x)
# axs[0].set_title('Predicted X-coordinate')
# axs[1].plot(t, corrected_x)
# axs[1].set_title('Corrected X-coordinate')
# axs[2].plot(t, uwb_x)
# axs[2].set_title('uwb X_coordinate')
# for ax in axs:
#     ax.set_xlabel('time')
# fig.tight_layout()

# plt.plot(t, pre_x, 'r--', t, corrected_x, 'b--', t, uwb_x, 'g--')

line, = plt.plot(t, pre_x, '-', lw=1, label='predicted_x')
line1, = plt.plot(t, corrected_x, '--', lw=1, label='corrected_x')
line2, = plt.plot(t, uwb_x, lw=1, label='uwb_x')
plt.xlabel("Time")
plt.ylabel("X-coordinate (m)")
plt.legend()
# plt.show()
plt.savefig('kf.pdf', dpi=300)
