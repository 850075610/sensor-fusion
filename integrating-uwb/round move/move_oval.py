#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from turtlesim.msg import Pose

vehicle_pose = PoseStamped()


def local_pose_callback(msg):
    global vehicle_pose
    vehicle_pose = msg


class Robot:

    def __init__(self):
        # Creating our node,publisher and subscriber
        rospy.init_node('robot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber(
            "/pose", PoseStamped, local_pose_callback, queue_size=10)
        self.pose = Pose()
        self.rate = rospy.Rate(50)

    # Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.theta = round(self.pose.theta, 4)
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def make_circle(self):
        # for the movement in lab, radius: 1, frequency: 0.1, turn: 1
        r = input("what is the radius ?: ")
        f = input("What is the frequency ?: ")
        n = input("How many turns ?: ")

        counter = 0

        goal_pose = Pose()
        vel_msg = Twist()
        self.rate.sleep()
        t0 = float(rospy.Time.now().to_sec())
        distance = 0

        angle = self.pose.theta

        while (2 * 3.1416 * r * n) >= distance:
            vel_msg.linear.x = 2 * 3.1416 * f * r
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 2 * 3.1416 * f
            # print(self.pose.x)
            t1 = float(rospy.Time.now().to_sec())
            distance = vel_msg.linear.x * (t1 - t0)
            print('No. of turned = ' + str(distance / (2 * 3.1416 * r)))

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        # Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        # print(distance//(2*3.1416*r))


if __name__ == '__main__':
    try:
        # Testing our function
        x = Robot()
        x.make_circle()

    except rospy.ROSInterruptException:
        pass
