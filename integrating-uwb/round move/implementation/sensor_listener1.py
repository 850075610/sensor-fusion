#!/usr/bin/env python
import argparse
import csv
import os.path
import random
import time

import rospy
from geometry_msgs.msg import AccelWithCovarianceStamped, Twist
from localizer_dwm1001.msg import Tag
from nav_msgs.msg import Odometry
from rosbot_ekf.msg import Imu
from tf.transformations import euler_from_quaternion

tag_pos = [0.0, 0.0]
args = None
res = [None] * 22
timeVariable = None


def write_to_csv(filename, dic):
    file_exists = os.path.isfile(filename)
    with open(filename, 'a') as csvfile:
        # headers = ['seq', 'secs', 'nsecs', 'x', 'y', 'z', 'w',
        #            'av_x', 'av_y', 'av_z', 'la_x', 'la_y', 'la_z', 'time']
        # headers = ['time', 'predicted_x', 'corrected_x', 'uwb_x', 'uwb_y',
        headers = ['time', 'la_x', 'la_y', 'uwb_x', 'uwb_y', 'vel_linear_x', 'vel_angular_z',
                   'mpu_ang_vel_z', 'mpu_linear_acc_x', 'mpu_linear_acc_y', 'odom_linear_x', 'odom_angular_z',
                   # start from index 12
                   'cmd_linear_x', 'cmd_linear_y', 'odom_yaw', 'cmd_angular_z',
                   # start from index 16
                   'var_x', 'var_y', 'var_xvel', 'var_yvel', 'vel_linear_y', 'odom_linear_y']

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


def local_uwb_callback(msg):
    global tag_pos
    tag_pos = [msg.x, msg.y]
    rospy.loginfo("tag_pos = %s", tag_pos)


def write():
    """Write to specified csv file with the frequency of 20Hz: dt = 0.05s"""
    # Make sure the timestamp has increased
    global timeVariable
    if args.save and args.save.endswith('.csv') and timeVariable != res[0]:
        timeVariable = res[0]
        write_to_csv(args.save, res)
    # else:
    #     current_date = datetime.datetime.now()
    #     # name the csv file according to date automatically
    #     filename = 'data' + str(current_date.day) + \
    #                str(current_date.month) + str(current_date.year)
    #     write_to_csv(str(filename + '.csv'), res)


def time_keeper(lost_by_reading, period=1):
    if lost_by_reading > period:
        pass  # a problem because reading takes longer than period
    elif lost_by_reading < period:
        time.sleep(period - lost_by_reading)


def mock_register_reading(precision=1000):
    rand_sleep = random.randint(0, 10) / precision
    time.sleep(rand_sleep)


_measure = time.time()

period = 0.05


def accel_callback(data):
    global tag_pos, _measure
    secs = data.header.stamp.secs
    nsecs = data.header.stamp.nsecs

    global res
    res[0] = secs + nsecs * 10 ** (-9)
    res[1] = data.accel.accel.linear.x
    res[2] = data.accel.accel.linear.y
    # res[3] = data.accel.accel.linear.z
    res[3:5] = tag_pos
    # write to specified csv file every 0.05s
    start = time.time()
    write()

    # just for log
    print ('measured after', time.time() - _measure, ', ERROR: ', period - (time.time() - _measure))
    _measure = time.time()
    # end

    finish = time.time()
    reading_time = finish - start
    time_keeper(reading_time, period=period)


def imu_callback(data):
    # 'Imu' object is not iterable
    # rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)
    global res
    # data.angular_velocity is tuple object
    res[7] = data.angular_velocity[2]
    # print "velocity angular y = " + str(data.angular_velocity.y)
    res[8] = data.linear_acceleration[0]
    res[9] = data.linear_acceleration[1]


def odom_callback(data):
    global res
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, res[14]) = euler_from_quaternion(orientation_list)
    res[10] = data.twist.twist.linear.x
    res[21] = data.twist.twist.linear.y
    res[11] = data.twist.twist.angular.z

    # pose.covariance must be stored given indices rather than a whole matrix
    # format according to
    # http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovariance.html

    pos_cov = data.pose.covariance
    # len(pos_cov) = 36: 6 variables:
    # x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis
    # rospy.loginfo("Length pos_cov = %s", len(pos_cov))
    # var_x
    res[16] = pos_cov[0]

    # var_y: pos_cov[1][1]
    res[17] = pos_cov[7]

    # var_xvel, var_yvel can be acquired from data.twist.covariance
    twist_cov = data.twist.covariance
    res[18] = twist_cov[0]
    # twist_cov[1][1]
    res[19] = twist_cov[7]


def odom_filtered_callback(data):
    global res
    # pos = data.pose.pose.position
    # distance since last filter cycle
    # motion = pos.x - last_odom_pos
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    # res[12] = data.twist.twist.linear.x
    # res[24] = data.twist.twist.linear.y
    # res[13] = data.twist.twist.angular.z
    # res[15] = yaw

    # rospy.loginfo(
    # "rospy.Time.now().to_sec() = %s, res[0] = %s", rospy.Time.now().to_sec(), res[0])
    # res[0] = rospy.Time.now().to_sec()
    # add kalman cycle here
    # mu, sig = predict_step(mu, sig, motion, motion_sig)
    # res[1] = mu
    # rospy.loginfo("predict step: [%s, %s]", mu, sig)
    # mu, sig = correct_step(mu, sig, tag_pos[0], measurement_sig)
    # res[2] = mu
    # rospy.loginfo("correct step: [%s, %s]", mu, sig)
    # res[3:5] = tag_pos
    # keep for next filter cycle
    # last_odom_pos = pos.x


def vel_callback(data):
    global res
    # data.linear.x is proven to approach assigned velocity as soon as possible
    res[5] = data.linear.x
    res[20] = data.linear.y
    res[6] = data.angular.z


def cmd_callback(data):
    global res
    # data.linear.x is proven to approach assigned velocity as soon as possible
    res[12] = data.linear.x
    res[13] = data.linear.y
    res[15] = data.angular.z


def callback(data):
    # 'Imu' object is not iterable
    # rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)
    print "velocity angular z = " + str(data.angular_velocity.z)
    print "velocity angular y = " + str(data.angular_velocity.y)
    print "acceleration linear x =" + str(data.linear_acceleration.x)
    print "acceleration linear y =" + str(data.linear_acceleration.y)
    # data = str(data)
    # res = re.findall(r"[-+]?\d*\.\d+|\d+", data)
    # # combine secs and nsecs to complete timestamp
    # res.append(float(res[1]) + float(res[2]) * 10 ** (-9))
    # if args.save and args.save.endswith('.csv'):
    #     write_to_csv(args.save, res)
    # else:
    #     current_date = datetime.datetime.now()
    #     # name the csv file according to date automatically
    #     filename = 'data' + str(current_date.day) + \
    #                str(current_date.month) + str(current_date.year)
    #     write_to_csv(str(filename + '.csv'), res)


def listener():
    rospy.init_node('listener', anonymous=True)
    # rospy.Subscriber("mpu9250", Imu, callback)
    rospy.Subscriber("/accel/filtered",
                     AccelWithCovarianceStamped, accel_callback)
    rospy.Subscriber("/dwm1001/tag", Tag,
                     local_uwb_callback, queue_size=10)
    rospy.Subscriber("mpu9250", Imu, imu_callback)
    rospy.Subscriber("odom", Odometry, odom_callback)
    # rospy.Subscriber("/odometry/filtered", Odometry, odom_filtered_callback)
    rospy.Subscriber("/velocity", Twist, vel_callback)
    rospy.Subscriber("/cmd_vel", Twist, cmd_callback)
    rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--save', required=False,
                        help="save to <filename>")
    args = parser.parse_args()
    listener()
