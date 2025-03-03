#!/usr/bin/env python
# coding=utf-8
import argparse
import csv
import math
import numpy as np
import os.path
import re
from datetime import datetime

import rospy
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from geometry_msgs.msg import Twist, PoseStamped
from localizer_dwm1001.msg import Tag
from nav_msgs.msg import Odometry


def local_uwb_callback(msg):
    global tag_pos, res
    tag_pos = [msg.x, msg.y]
    res[5] = msg.x
    res[6] = msg.y
    # rospy.loginfo(rospy.get_caller_id() + " tag_pos:  %s", tag_pos)


rospy.Subscriber("/dwm1001/tag", Tag, local_uwb_callback, queue_size=10)

args = None
vel_msg = None
vehicle_pose = PoseStamped()
v_pose = [None, None]
tag_pos = [None, None]
cur_pos = [None, None]
res = [None] * 8


def local_pose_callback(msg):
    global vehicle_pose
    vehicle_pose = msg


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)
    data = str(data)
    res = re.findall(r"[-+]?\d*\.\d+|\d+", data)
    # rospy.loginfo(rospy.get_caller_id() + " x = %s, y = %s", res[3], res[4])


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
        headers = ['time', 'speed', 'distance', 'x',
                   'y', 'uwb_x', 'uwb_y', 'time_in_sec']
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
    """ Method to move the robot straight."""

    rospy.Subscriber(
        "/pose", PoseStamped, local_pose_callback, queue_size=10)

    # rospy.spin()
    global vel_msg, res, v_pose
    vel_msg = Twist()
    # 0.1 is a safe value for speed. Attention: speed should <= 0.5
    speed = speed

    # Receiving the user's input
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
    global tag_pos, cur_pos
    ini_pos = tag_pos
    rospy.loginfo(rospy.get_caller_id() + " Initial pose from tag: %s", ini_pos)
    # without this while loop, the robot doesn't move at all!
    # while not rospy.is_shutdown():
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    res[7] = t0
    a = datetime.now()
    res[0] = a.strftime("%Y-%m-%d %H:%M:%S")
    res[1] = speed
    res[2] = distance
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
        rospy.Subscriber("/odometry/filtered", Odometry, callback)
        rospy.Subscriber("/dwm1001/tag", Tag,
                         local_uwb_callback, queue_size=10)
        # Publish the velocity
        velocity_publisher.publish(vel_msg)
        # Takes actual time to velocity calculus
        t1 = rospy.Time.now().to_sec()
        # time interval
        dt = t1 - t0

        def f_cv(x, dt):
            """ state transition function for a
            constant velocity mobile robot"""
            # Newtonian equations xk = xk-1 + v(xk-1) * dt
            # yk = yk-1 + v(yk-1) * dt
            F = np.array([[1., dt, 0, 0],  # x = x0 + dx*dt
                          [0., 1., 0, 0],  # dx = dx0
                          [0., 0, 1, dt],  # y = y0+dy*dt
                          [0., 0, 0, 1]])  # dy = dy0
            # The dot product of 2 vectors is the sum of the element-wise multiplications of each element.
            return np.dot(F, x)

        def h_cv(x):
            return x[[0, 2]]

        ukf = UKF(dim_x=4, dim_z=2, fx=f_cv,
                  hx=h_cv, dt=dt, points=sigmas)
        ukf.x = np.array([tag_pos[0], speed, tag_pos[1], 0])
        # The UWB readings are in meters with an error of σ = 0.3 meters in both x and y. This gives us a
        # measurement noise matrix of
        ukf.R = np.diag([std_x ** 2, std_y ** 2])
        ukf.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=dt, var=0.02)
        ukf.Q[2:4, 2:4] = Q_discrete_white_noise(2, dt=dt, var=0.02)

        # Bu is control input
        # kf.B = np.array([[0, dt, 0, 0]]).T
        # assume that the process noise can be represented by the discrete white noise model - that is,
        # that over each time period the acceleration is constant.
        # kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=q_var)
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
        res[7] = t1
        # Calculates distancePoseStamped
        current_distance = speed * dt
        # subtract v_pose for calibration
        # x = (ini_pos[0] + current_distance + np.random.normal(0, scale_factor * current_distance)) * 0.1 + tag_pos[
        #     0] * 0.9
        # y = (ini_pos[1] + np.random.normal(0, scale_factor * current_distance)) * 0.1 + tag_pos[1] * 0.9
        x = ukf.x.copy()[0]
        y = ukf.x.copy()[2]
        res[3] = x
        res[4] = y
        cur_pos = [x, y]
        ukf.predict()
        cur_pos_arr = np.array(cur_pos)
        ukf.update(cur_pos_arr)
        uxs.append(ukf.x.copy())
        rospy.loginfo("ukf.x = %s", ukf.x)
        rospy.loginfo(rospy.get_caller_id() + " Current pose based on UKF: %s", cur_pos)
        # if x not in res and y not in res:
        #     res.extend(cur_pos)
        if all(res[i] for i in range(len(res))):
            # no element is None/empty
            if args.save and args.save.endswith('.csv'):
                write_to_csv(args.save, res)
            else:
                current_date = datetime.now()
                # name the csv file according to date automatically
                filename = 'poseRecord' + str(current_date.day) + \
                           str(current_date.month) + str(current_date.year)
                write_to_csv(str(filename + '.csv'), res)
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
        r.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--save', required=False,
                        help="save to <filename>")
    args = parser.parse_args()
    # Starts a new node
    rospy.init_node('robot', anonymous=True, disable_signals=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    r = rospy.Rate(0.5)  # 0.5hz
    try:
        # Testing move() function
        for _ in range(2):
            #     move(0.1, 2, False)
            #     r.sleep()
            #     rotate(degree2radians(10), degree2radians(90), False)
            #     r.sleep()
            move(0.5, 2, True)
            move(0.5, 2, False)
        move(0.5, 2, True)
        rospy.on_shutdown(stop)
        rospy.signal_shutdown("You pressed Ctrl+C!")
    except rospy.ROSInterruptException:
        stop()
