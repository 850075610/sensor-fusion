import pandas as pd
from sklearn.metrics import mean_squared_error
# read from csv and save corresponding column data to variables

df = pd.read_csv('pose1204.csv')
# modify the path if needed
df1 = pd.read_csv('2104 with 3 calibrated working anchor/pose2104.csv')
# saved_column = df.column_name  # can also use df['column_name']
# x and y are coordinates based on calculation
y1 = df1.y
uwb_y1 = df1.uwb_y
y = df.y
uwb_y = df.uwb_y

print("MSE for 1st. run =", mean_squared_error(y, uwb_y))
print("MSE for 2nd. run =", mean_squared_error(y1, uwb_y1))
