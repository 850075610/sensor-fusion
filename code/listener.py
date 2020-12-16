#!/usr/bin/env python
import re
import os.path
import csv
import rospy
from std_msgs.msg import String
from rosbot_ekf.msg import Imu
from nav_msgs.msg import Odometry

def write_to_csv(filename, dic):
    file_exists = os.path.isfile(filename)
    # print "File exists: ", file_exists
    with open(filename, 'a') as csvfile:
        headers = ['seq', 'secs', 'nsecs', 'x', 'y', 'z', 'w',
                   'av_x', 'av_y', 'av_z', 'la_x', 'la_y', 'la_z']
        file_is_empty = os.stat(filename).st_size == 0
        header_writer = csv.writer(csvfile, lineterminator='\n')
        writer = csv.DictWriter(csvfile,
                                delimiter=',',
                                lineterminator='\n',
                                fieldnames=headers)

        if file_is_empty:
            header_writer.writerow(headers)
            # writer.writeheader()  # file doesn't exist yet, write a header

        writer.writerow(
            {headers[i]: dic[i]
             for i in range(len(dic))})

def callback(data):
    # 'Imu' object is not iterable
    # rospy.loginfo(rospy.get_caller_id() + " Type of data: %s", type(data))
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)
    data = str(data)
    # lin_acc_ind = data.find('linear_acceleration')
    # There are 3 values indicating linear acceleration
    # rospy.loginfo(rospy.get_caller_id() + " Index for linear acceleration is: %s", lin_acc_ind)
    res = re.findall(r"[-+]?\d*\.\d+|\d+", data)
    write_to_csv('data.csv', res)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("mpu9250", Imu, callback)
    # rospy.Subscriber("odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
