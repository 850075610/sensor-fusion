# Data Explanation

## topic explanation
* topic */accel/filtered* and */odometry/filtered* are published by the *ekf* of *odometry* and *IMU*

* topic *mpu9250* contains all raw imu data

## column explanation

* 'time': timestamp  
* 'la_x', 'la_y': linear acceleration x and y from topic */accel/filtered*  
* 'uwb_x', 'uwb_y': from topic */dwm1001/tag*  
* 'vel_linear_x', 'vel_angular_z': linear velocity x and angular velocity z from topic */velocity*
* 'mpu_ang_vel_z', 'mpu_linear_acc_x', 'mpu_linear_acc_y': angular_velocity_z  , linear_acceleration_x, linear_acceleration_y from topic *mpu9250*  
* 'odom_linear_x', 'odom_angular_z', 'odom_yaw': linear_velocity_x, angular_velocity_z and yaw from topic *odom*  
* 'odom_filtered_linear_x', 'odom_filtered_angular_z','odom_filtered_yaw': linear_velocity_x, angular_velocity_z and yaw from topic */odometry/filtered*  