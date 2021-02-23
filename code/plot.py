import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from pylab import *
import time

"""
Robot running process plot
@author: Mingze
@contact: mingzec@gmail.com
@date: 02.2021
"""

# read from csv and save corresponding column data to variables
df = pd.read_csv('poseData.csv')
# saved_column = df.column_name  # can also use df['column_name']
# x and y are coordinates, (x = 0, y = 0) is where the robot was first
# turned on.
x = df.pos_x
y = df.pos_y
ori_z = df.ori_z
ori_w = df.ori_w

xm = np.min(x) - 0.5
xM = np.max(x) + 0.5
ym = np.min(y) - 0.5
yM = np.max(y) + 0.5

# fig = px.line(df, x, y,
#               title='Robot\'s coordinates over time')
# parameter duration: the smaller, the faster the animation is
fig = go.Figure(
    # name='trace' can be a param inside go.Scatter
    data=[go.Scatter(x=x, y=y,
                     mode="lines",
                     line=dict(width=2, color="blue")),
          go.Scatter(x=x, y=y,
                     mode="lines",
                     line=dict(width=2, color="blue"))],
    layout=go.Layout(
        xaxis=dict(title='pos_x', range=[xm, xM],
                   autorange=False, zeroline=False),
        yaxis=dict(title='pos_y', range=[ym, yM],
                   autorange=False, zeroline=False),
        title_text="Robot\'s coordinates over time", hovermode="closest",
        updatemenus=[dict(type="buttons",
                          buttons=[dict(args=[None, {"frame": {"duration": 15,
                                                               "redraw": False},
                                                     "fromcurrent": True,
                                                     "transition": {"duration": 0}}],
                                        label="Play",
                                        method="animate"
                                        )])]),
    frames=[go.Frame(
        data=[go.Scatter(
            x=[x[k]],
            y=[y[k]],
            mode="markers",
            name='robot in the trace',
            marker=dict(color="red", size=10))])
            for k in range(len(df))]
)

fig.add_trace(go.Scatter(
    x=[x[0], x[len(df) - 1]],
    y=[y[0], y[len(df) - 1]],
    mode="markers+text",
    name="Start and End",
    text=["Start", "End"],
    textposition="top center"
))

fig.show()
