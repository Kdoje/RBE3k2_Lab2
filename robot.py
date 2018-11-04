#!/usr/bin/env python

import math
import copy
import numpy
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseWithCovariance
from geometry_msgs.msg import Quaternion
import tf
from tf.transformations import euler_from_quaternion


class Robot:
    px = 0
    py = 0
    zRot = 0

    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg=Twist()

    def __init__(self):
        """"
        Set up the node here
        """
        rospy.loginfo('hello')
        sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """
        now = rospy.Time.now()
        self._odom_list.waitForTransform('base_link', 'odom', now, rospy.Duration(1))
        transGoal = self._odom_list.transformPose('base_link', goal)

        (goal_angle_roll, goal_angle_pitch, goal_angle_yaw) = tf.transformations.euler_from_quaternion(
            [transGoal.pose.orientation.x,
             transGoal.pose.orientation.y,
             transGoal.pose.orientation.z,
             transGoal.pose.orientation.w])

    def drive_straight(self, speed, distance):
        """
        Make the turtlebot drive straight
        :type speed: float
        :type distance: float
        :param speed: speed to drive
        :param distance: distance to drive
        :return:
        """

        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        rospy.loginfo("the delta distance is {}".format(self.px))

        while (distance-self.px) > .01:  # this will track the change in distance and
                                                  # and move it forward
            self.vel_msg.linear.x = speed
            self.vel_pub.publish(self.vel_msg)
            rospy.loginfo("running drive straight")
            rospy.loginfo("my x position is {} and rotation {}".format(self.px, self.zRot))

        while (distance-self.px) < .01:
            self.vel_msg.linear.x = -speed
            self.vel_pub.publish(self.vel_msg)
            rospy.loginfo("running drive straight")
            rospy.loginfo("my x position is {} and rotation {}".format(self.px, self.zRot))

        self.vel_msg.linear.x = 0
        self.vel_pub.publish(self.vel_msg)

    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """

    def odom_callback(self, msg):
        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, self.zRot = euler_from_quaternion(q)
        # rospy.loginfo("hello from odom")
        # rospy.loginfo("my roll is {}".format(self.px))


if __name__ == '__main__':
    rospy.init_node('Robot')
    ne = Robot()
    ne.drive_straight(.2, 0)
    while not rospy.is_shutdown():
        rate = rospy.Rate(10)  # 10hz
        rate.sleep()
    pass
