# Sensor-fusion
## Timeline

Mar. 18th. 2021: Integrated UWB  
Mar. 25th. 2021: Conducted [round movement](https://github.com/850075610/sensor-fusion/blob/main/integrating-uwb/round%20move/round_move.py)  
Apr. 12th. 2021: Implemented [UKF on round movement](https://github.com/850075610/sensor-fusion/blob/main/integrating-uwb/round%20move/UKF/round_move_ukf.py)  
Apr. 21th. 2021: Calibrated UWB anchors with *BOSCH Laser measure* and run UKF on round movement with 3 stable working anchors only  
May. 7th. 2021: Added calculation of [process error and measurement error](https://github.com/850075610/sensor-fusion/tree/main/data)  
Jun. 2nd. 2021: Implemented one dimensional [kalman filter](https://github.com/850075610/sensor-fusion/tree/main/integrating-uwb/round%20move/KF)  
Jun. 4th. 2021: Gathered sensor data into *[sensorData.csv](https://github.com/850075610/sensor-fusion/blob/main/integrating-uwb/round%20move/KF/sensorData.csv)*  
Jun. 11th. 2021: Implemented two dimensional kalman filter  
Jun. 22th. 2021: Implemented particle filter for straight movement  
Jun. 24th. 2021: Updated kalman filter with control input alone  
Jun. 25th. 2021: Added ground truth and repetitive experiment result as evaluation  
Jul. 5th. 2021: Updated evaluation with mean error and its variance  