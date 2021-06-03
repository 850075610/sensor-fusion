#!/usr/bin/env python
# coding=utf-8

"""Apply Kalman Filter to round movement."""

from __future__ import division

import argparse
import csv
import math
import os.path
from datetime import datetime

import numpy as np
import rospy
from filterpy.kalman import ExtendedKalmanFilter as EKF
from filterpy.kalman import MerweScaledSigmaPoints
from geometry_msgs.msg import Twist, PoseStamped
from localizer_dwm1001.msg import Tag
from nav_msgs.msg import Odometry
from numpy import array
from tf.transformations import euler_from_quaternion

from kalman_filter import predict_step, correct_step

# global variables
last_odom_pos = 0.0
vehicle_pose = PoseStamped()
vel_msg = None
# keep last measurement from UWB tag:
tag_pos = [0.0, 0.0]
res = [None] * 5
# initial state
mu = 2.0
sig = 1000

# initial motion sigma derived from the experiments (0.028 and 0.077 are the theoretical standard
# deviation of the motions normal distribution derived from extensive
# experiments)
motion_sig = 0.028 ** 2
# alternative measurement_sig = 0.3 ** 2
measurement_sig = 0.3 ** 2
# distance since last filter cycle
motion = 0.0

args = None

v_pose = [None, None]
odom_pos = [None, None]
cur_pos = [None, None]


def residual(a, b):
    """ compute residual (a-b) between measurements containing
[range, bearing]. Bearing is normalized to [-pi, pi)"""
    y = a - b
    y[1] = y[1] % (2 * np.pi)
    if y[1] > np.pi:
        y[1] -= 2 * np.pi
    return y


def H_of(x, landmark_pos):
    """ compute Jacobian of H matrix where h(x) computes
the range and bearing to a landmark for state x """
    px = landmark_pos[0]
    py = landmark_pos[1]
    hyp = (px - x[0, 0]) ** 2 + (py - x[1, 0]) ** 2
    dist = math.sqrt(hyp)
    H = array(
        [[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0],
         [(py - x[1, 0]) / hyp, -(px - x[0, 0]) / hyp, -1]])
    return H


def hx(x, landmark_pos):
    """ takes a state variable and returns the measurement
that would correspond to that state.
"""
    px = landmark_pos[0]
    py = landmark_pos[1]
    dist = math.sqrt((px - x[0, 0]) ** 2 + (py - x[1, 0]) ** 2)

    Hx = array([[dist], [math.atan2(py - x[1, 0], px - x[0, 0]) - x[2, 0]]])
    return Hx


def ekf_update(ekf, z, landmark):
    ekf.update(z, HJacobian=H_of, Hx=hx, residual=residual, args=landmark, hx_args=landmark)


def local_uwb_callback(msg):
    global tag_pos
    tag_pos = [msg.x, msg.y]
    # res[3] = msg.x
    # res[4] = msg.y
    # rospy.loginfo(rospy.get_caller_id() + " tag_pos:  %s", tag_pos)


def callback(data):
    global res, tag_pos, mu, sig, motion_sig, last_odom_pos, measurement_sig, filter_pub, roll, pitch, yaw, vel_x
    pos = data.pose.pose.position
    # distance since last filter cycle
    motion = pos.x - last_odom_pos
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
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
    if all(res[i] for i in range(len(res))):
        # no element is None/empty
        if args.save and args.save.endswith('.csv'):
            write_to_csv(args.save, res)
        else:
            current_date = datetime.now()
            # name the csv file according to date automatically
            filename = 'kf' + str(current_date.day) + \
                       str(current_date.month) + str(current_date.year)
            write_to_csv(str(filename + '.csv'), res)
    # keep for next filter cycle
    last_odom_pos = pos.x
    rospy.loginfo("mu = %s, tag_x = %s", mu, tag_pos[0])
    # publish filtered position estimate
    # filter_pub.publish(mu)
    # Use rqt_plot command to generate data from a rostopic


rospy.Subscriber("/odometry/filtered", Odometry, callback)
rospy.Subscriber("/dwm1001/tag", Tag, local_uwb_callback, queue_size=10)


def local_pose_callback(msg):
    global vehicle_pose
    vehicle_pose = msg


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
    res.extend(end_pose)
    print "Stop pose: ", end_pose


def write_to_csv(filename, dic):
    # rospy.loginfo(len(dic))
    file_exists = os.path.isfile(filename)
    with open(filename, 'a') as csvfile:
        # robot always start from [0, 0] + ini_pos
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


class RobotEKF(EKF):
    def __init__(self, dt, wheelbase, std_vel):
        EKF.__init__(self, 3, 2, 2)
        # self.std_steer = None
        self.a = 0
        self.theta = 0
        self.dt = dt
        # self.wheelbase = wheelbase
        self.std_vel = std_vel
        # self.std_steer = std_steer

    def move(self, speed, distance, forward):
        """ Method to move the robot straight."""

        rospy.Subscriber(
            "/pose", PoseStamped, local_pose_callback, queue_size=10)

        # rospy.spin()
        global vel_msg, res, v_pose
        vel_msg = Twist()
        # 0.5 is a safe value for speed. Attention: speed should <= 0.5
        speed = speed

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
        # Only for calibration
        rospy.Subscriber("/dwm1001/tag", Tag, local_uwb_callback, queue_size=10)
        # rospy.loginfo(rospy.get_caller_id() + " tag = %s", tag)
        global tag_pos, cur_pos, odom_pos
        ini_pos = tag_pos
        rospy.loginfo(rospy.get_caller_id() + " Initial pose from tag: %s", ini_pos)
        # without this while loop, the robot doesn't move at all!
        # while not rospy.is_shutdown():
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        a = datetime.now()
        # res[0] = a.strftime("%Y-%m-%d %H:%M:%S")
        # res[1] = speed
        # res[2] = distance
        current_distance = 0

        # Loop to move the robot an specified distance
        # distance = speed * time
        sigmas = MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=1.)
        std_x, std_y = .3, .3
        # kf = KalmanFilter(4, 2)

        # UWB provides position but not velocity, so the measurement function:
        # kf.H = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
        # kf.P = np.eye(2) * 50

        q_var = 0.02
        uxs = []
        while current_distance < distance:
            # Publish the velocity
            velocity_publisher.publish(vel_msg)
            # Takes actual time to velocity calculus
            t1 = rospy.Time.now().to_sec()
            # time interval
            dt = t1 - t0
            res[0] = t1
            # Calculates distancePoseStamped
            current_distance = speed * dt
            # subtract v_pose for calibration
            # x = (ini_pos[0] + current_distance + np.random.normal(0, scale_factor * current_distance)) * 0.1 + tag_pos[
            #     0] * 0.9
            # y = (ini_pos[1] + np.random.normal(0, scale_factor * current_distance)) * 0.1 + tag_pos[1] * 0.9
            # x = ekf.x.copy()[0]
            # y = ekf.x.copy()[2]
            # rospy.loginfo(rospy.get_caller_id() + " Pose from odom: %s, tag = %s", odom_pos, tag_pos)
            # res[1] = ini_pos[0] + odom_pos[0]
            # res[2] = ini_pos[1] + odom_pos[1]
            res[3] = tag_pos[0]
            res[4] = tag_pos[1]
            # cur_pos = [res[1], res[2]]
            # rospy.loginfo(rospy.get_caller_id() + " Current pose based on EKF: %s", cur_pos)
            # if x not in res and y not in res:
            #     res.extend(cur_pos)
            # if all(res[i] for i in range(len(res))):
            #     # no element is None/empty
            #     if args.save and args.save.endswith('.csv'):
            #         write_to_csv(args.save, res)
            #     else:
            #         current_date = datetime.now()
            #         # name the csv file according to date automatically
            #         filename = 'poseRecord' + str(current_date.day) + \
            #                    str(current_date.month) + str(current_date.year)
            #         write_to_csv(str(filename + '.csv'), res)
            # rospy.loginfo(rospy.get_caller_id() + " x = %s", current_distance)
            # After the loop, stops the robot
        vel_msg.linear.x = 0
        # Force the robot to stop
        velocity_publisher.publish(vel_msg)

        # turn 90 degree counter clockwise
        # rotate(math.pi / 4, 90, 0)


def rotate(angular_speed, angle, clockwise):
    """Rotate the robot."""
    # angular_speed: degree/sec
    # counter-clockwise: turn left: positive angle
    # clockwise: turn right: negative angle
    global velocity_publisher, vel_msg
    vel_msg = Twist()
    # set linear velocity in the x-axis
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    # set angular velocity in the y-axis: degree/sec
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = - \
        abs(angular_speed) if clockwise else abs(angular_speed)
    current_angle = 0.0
    t0 = rospy.Time.now().to_sec()
    r = rospy.Rate(10)  # 10hz
    rel_angle = angle * 2 * math.pi / 360
    while current_angle < rel_angle:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)
        rospy.loginfo("current angle = %s", current_angle)
        r.sleep()
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


def degree2radians(angle_in_degrees):
    return math.radians(angle_in_degrees)


def get_distance(p1, p2):
    return math.sqrt(((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2))


def move_to_goal(goal, distance_tolerance):
    global velocity_publisher, vel_msg, cur_pos
    vel_msg = Twist()
    r = rospy.Rate(10)  # 10hz
    while get_distance(cur_pos, [goal.x, goal.y]) > distance_tolerance:
        # set linear velocity in the x-axis
        vel_msg.linear.x = 1.5 * get_distance(cur_pos, [goal.x, goal.y])
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        # set angular velocity in the z-axis: degree/sec
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 4 * \
                            (math.atan2(goal.y - cur_pos[1], goal.x - cur_pos[0]))
        velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--save', required=False,
                        help="save to <filename>")
    args = parser.parse_args()
    # Starts a new node
    rospy.init_node('round_move_listener', anonymous=True, disable_signals=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/odometry/filtered", Odometry, callback)
    rospy.Subscriber("/dwm1001/tag", Tag,
                     local_uwb_callback, queue_size=10)
    r = rospy.Rate(0.5)  # 0.5hz
    dt = 0.07075
    robot = RobotEKF(wheelbase=0.105, std_vel=0.5, dt=dt)
    try:
        # Testing move() function
        for _ in range(2):
            #     move(0.1, 2, False)
            #     r.sleep()
            #     rotate(degree2radians(10), degree2radians(90), False)
            #     r.sleep()
            robot.move(speed=0.5, distance=2, forward=True)
            r.sleep()
            robot.move(speed=0.5, distance=2, forward=False)
        # move(0.5, 2, True)
        rospy.on_shutdown(stop)
        rospy.signal_shutdown("You pressed Ctrl+C!")
    except rospy.ROSInterruptException:
        stop()
