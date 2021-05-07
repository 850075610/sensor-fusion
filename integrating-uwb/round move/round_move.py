#!/usr/bin/env python
import argparse
import csv
import math
import os.path
import re
import sys
from datetime import datetime

import rospy
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

    pose_sub = rospy.Subscriber(
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
    print "Initial pose from tag: ", ini_pos
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
    while current_distance < distance:
        rospy.Subscriber("/odometry/filtered", Odometry, callback)
        rospy.Subscriber("/dwm1001/tag", Tag,
                         local_uwb_callback, queue_size=10)
        # Publish the velocity
        velocity_publisher.publish(vel_msg)
        # Takes actual time to velocity calculus
        t1 = rospy.Time.now().to_sec()
        res[7] = t1
        # Calculates distancePoseStamped
        current_distance = speed * (t1 - t0)
        # subtract v_pose for calibration
        x = float(vehicle_pose.pose.position.x) + ini_pos[0] - v_pose[0]
        y = float(vehicle_pose.pose.position.y) + ini_pos[1] - v_pose[1]
        res[3] = x
        res[4] = y
        cur_pos = [x, y]
        print "Current pose: ", cur_pos
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
    """Return the pathname of the KOS root directory."""
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
        # Test move() function
        for _ in range(2):
            #     move(0.1, 2, False)
            #     r.sleep()
            #     rotate(degree2radians(10), degree2radians(90), False)
            #     r.sleep()
            move(0.5, 2, True)
            move(0.5, 2, False)
        # move(0.5, 2, True)
        rospy.on_shutdown(stop)
        rospy.signal_shutdown("You pressed Ctrl+C!")
    except rospy.ROSInterruptException:
        stop()
