#!/usr/bin/env python
"""
Date: May 2021
"""
import argparse
import csv
import os
from datetime import datetime

import rospy
from geometry_msgs.msg import Twist, AccelWithCovarianceStamped, PoseStamped
from localizer_dwm1001.msg import Tag
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

from round_move_kf import local_pose_callback

# global variables
last_odom_pos = 0.0
vehicle_pose = PoseStamped()
# keep last measurement from UWB tag:
tag_pos = [0.0, 0.0]
res = [None] * 5
# initial state
mu = 5.0
sig = 1000

# initial motion sigma derived from the experiments (the standard deviation of the motions normal distribution)
motion_sig = 0.028
measurement_sig = 0.077
# distance since last filter cycle
motion = 0.0


def local_uwb_callback(msg):
    global tag_pos, res
    tag_pos = [msg.x, msg.y]
    res[3] = msg.x
    res[4] = msg.y
    # rospy.loginfo(rospy.get_caller_id() + " tag_pos:  %s", tag_pos)


def accel_callback(data):
    # rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)
    acc_x = data.accel.accel.linear.x
    # data = str(data)
    # res = re.findall(r"[-+]?\d*\.\d+|\d+", data)
    # rospy.loginfo("Acceleration callback: x = %s, y = %s", res[3], res[4])


def odom_callback(data):
    global res, tag_pos, mu, sig, motion_sig, last_odom_pos, measurement_sig, filter_pub, roll, pitch, yaw, vel_x
    pos = data.pose.pose.position
    # distance since last filter cycle
    motion = pos.x - last_odom_pos
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    vel_x = data.twist.twist.linear.x
    res[0] = rospy.Time.now().to_sec()
    #     add kalman cycle here
    mu, sig = predict_step(mu, sig, motion, motion_sig)
    res[1] = mu
    # rospy.loginfo("predict step: [%s, %s]", mu, sig)
    mu, sig = correct_step(mu, sig, tag_pos[0], measurement_sig)
    res[2] = mu
    # rospy.loginfo("correct step: [%s, %s]", mu, sig)
    res[3:] = tag_pos
    # if all(res[i] for i in range(len(res))):
    #     # no element is None/empty
    #     if args.save and args.save.endswith('.csv'):
    #         write_to_csv(args.save, res)
    #     else:
    #         current_date = datetime.now()
    #         # name the csv file according to date automatically
    #         filename = 'kf' + str(current_date.day) + \
    #                    str(current_date.month) + str(current_date.year)
    #         write_to_csv(str(filename + '.csv'), res)
    # keep for next filter cycle
    last_odom_pos = pos.x
    rospy.loginfo("mu = %s, tag_x = %s", mu, tag_pos[0])
    # publish filtered position estimate
    filter_pub.publish(mu)
    # Use rqt_plot command to generate data from a rostopic


def stop():
    global vel_msg
    if vel_msg:
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
    global vehicle_pose
    global tag_pos
    end_pose = tag_pos
    # res.extend(end_pose)
    print "Stop pose: ", end_pose


# correct step function
def correct_step(mean1, var1, mean2, var2):
    """
    This function takes in two means and two squared variance terms, and return updated gaussian parameters.
    """
    # calculate the new gaussian parameters
    new_mean = (var1 * mean2 + var2 * mean1) / (var1 + var2)
    # also equals to var1 * var2 /(var1  +  var2)
    # TODO: should from future import __division__?
    new_var = 1 / (1 / var1 + 1 / var2)
    return new_mean, new_var


def predict_step(mean1, var1, mean2, var2):
    global new_mean, new_var
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return new_mean, new_var


def write_to_csv(filename, dic):
    # rospy.loginfo(len(dic))
    file_exists = os.path.isfile(filename)
    with open(filename, 'a') as csvfile:
        # robot always start from [0, 0] + ini_pos
        # vel_x should also be gathered
        headers = ['time', 'predicted_x', 'corrected_x', 'uwb_x', 'uwb_y']
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


def move(speed, distance, forward):
    """ Method to move the robot straight forward/backward."""

    rospy.Subscriber(
        "/pose", PoseStamped, local_pose_callback, queue_size=10)

    # rospy.spin()
    global v_pose
    vel_msg = Twist()
    # Attention: speed should <= 0.5 m/s for safety!
    speed = speed

    # Receiving user's input
    # forward = input("Forward?: ")  # True or False
    forward = forward
    # Due to the limitation of experimental environment, distance should be <=
    # 2m
    # distance = float(input("How many meters do you want the robot to move?: "))
    distance = distance
    scale_factor = 0.06
    rospy.loginfo(rospy.get_caller_id() + "--- Moving the robot ---")

    # Check if the movement is forward or backwards
    if forward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    # Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    x = float(vehicle_pose.pose.position.x)
    y = float(vehicle_pose.pose.position.y)
    v_pose = [x, y]
    rospy.loginfo("vehicle_pose: %s, %s", x, y)
    rospy.Subscriber("/dwm1001/tag", Tag, local_uwb_callback, queue_size=10)
    # rospy.loginfo(rospy.get_caller_id() + " tag = %s", tag)
    global tag_pos, cur_pos
    ini_pos = tag_pos
    print "Initial pose from tag: ", ini_pos
    # without this while loop, the robot doesn't move at all!
    while not rospy.is_shutdown():
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        # res[7] = t0
        a = datetime.now()
        # res[0] = a.strftime("%Y-%m-%d %H:%M:%S")
        # res[1] = speed
        # res[2] = distance
        current_distance = 0

        # Loop to move the robot an specified distance
        # distance = speed * time
        while current_distance < distance:
            # Publish the velocity
            velocity_publisher.publish(vel_msg)
            # Takes actual time to velocity calculus
            t1 = rospy.Time.now().to_sec()
            # time interval
            dt = t1 - t0
            # initial state (location and velocity)
            # ini_s = matrix([[ini_pos[0]], [ini_pos[1]], [0.], [0.]])
            # u = matrix([[0.], [0.], [0.], [0.]])  # external motion
            # # initial uncertainty: 0 for positions x and y, 1000 for the two velocities
            # P = matrix([[0., 0., 0., 0.], [0., 0., 0., 0.], [
            #     0., 0., 1000., 0.], [0., 0., 0., 1000.]])
            # # state transition matrix/next state function: generalize the 2d version to 4d
            # F = matrix([[1., 0, dt, 0], [0, 1., 0, dt], [0, 0, 1., 0], [0, 0, 0, 1.]])
            # # measurement function: reflect the fact that we observe x and y and the two velocities
            # H = matrix([[1., 0., 0., 0], [0., 1., 0., 0.], [0., 0., speed, 0.], [0, 0, 0, 0]])
            # # measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
            # R = matrix([[0.1, 0.], [0., 0.1]])
            # I = matrix([[1., 0, 0, 0], [0, 1., 0, 0], [0, 0, 1., 0],
            #             [0, 0, 0, 1.]])  # 4d identity matrix
            #
            # filter(ini_s, P)
            # res[7] = t1
            # Calculates distancePoseStamped
            current_distance = speed * dt
            # subtract v_pose for calibration
            # x = (ini_pos[0] + current_distance + np.random.normal(0, scale_factor * current_distance)) * 0.1 + tag_pos[
            #     0] * 0.9
            # y = (ini_pos[1] + np.random.normal(0, scale_factor *
            # current_distance)) * 0.1 + tag_pos[1] * 0.9
            # res[3] = x
            # res[4] = y
            # cur_pos = [x, y]
            # rospy.loginfo("Current pose: %s", cur_pos)
            # if x not in res and y not in res:
            #     res.extend(cur_pos)
            # rospy.loginfo(rospy.get_caller_id() + " x = %s", current_distance)
            # After the loop, stops the robot
        vel_msg.linear.x = 0
        # Force the robot to stop
        velocity_publisher.publish(vel_msg)
        r.sleep()
    # turn 90 degree counter clockwise
    # rotate(math.pi / 4, 90, 0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--save', required=False,
                        help="save to <filename>")
    args = parser.parse_args()
    try:
        # Starts a new node
        rospy.init_node('kalman_filter', anonymous=True, disable_signals=True)
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        filter_pub = rospy.Publisher('filtered_pos', Float64, queue_size=10)
        rospy.Subscriber("/dwm1001/tag", Tag, local_uwb_callback, queue_size=10)
        sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, odom_callback)
        rospy.Subscriber("/accel/filtered", AccelWithCovarianceStamped, accel_callback)
        rospy.Subscriber("/pose", PoseStamped, local_pose_callback, queue_size=10)
        r = rospy.Rate(0.5)  # 0.5hz
        # print node start information
        print '*' * 40
        print "Started Kalman Filter node"
        print '*' * 40
        # Test move() function
        for _ in range(2):
            #     move(0.1, 2, False)
            #     r.sleep()
            #     rotate(degree2radians(10), degree2radians(90), False)
            #     r.sleep()
            move(0.5, 1, True)
            r.sleep()
            move(0.5, 1, False)

        # move(0.5, 2, True)
        rospy.on_shutdown(stop)
        rospy.signal_shutdown("You pressed Ctrl+C!")
    except rospy.ROSInterruptException:
        stop()
