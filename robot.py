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
        goalSub = rospy.Subscriber("/move_base/goal", PoseStamped, self.nav_to_pose)

    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """
        rospy.loginfo('we movin')
        to_move_x = goal.pose.position.x
        to_move_y = goal.pose.position.y
        disp_x = to_move_x-self.px
        disp_y = to_move_y-self.py
        total_dist = math.sqrt(math.pow(disp_x, 2)+math.pow(disp_y, 2))
        # if disp_x<0:
        #     total_dist=-total_dist # make the distance negative
        init_rotation = math.atan2(disp_y, disp_x)

        # get the robot in position to move
        self.rotate(init_rotation)
        # actually move
        rospy.loginfo("actually moving straight for {}".format(total_dist))
        self.drive_straight(.5, total_dist)
        # do the last rotation
        quat = goal.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, to_rotate = euler_from_quaternion(q)
        self.rotate(to_rotate)
        # rospy.loginfo('the setpoint is x: {} y: {}'.format())
        # to_move=goal.pose.position.x
        # quat = goal.pose.orientation
        # q = [quat.x, quat.y, quat.z, quat.w]
        # roll, pitch, to_rotate = euler_from_quaternion(q)
        # self.rotate(to_rotate)
        # self.drive_straight(.5, to_move)
        # now = rospy.Time.now()
        # self._odom_list.waitForTransform('base_link', 'odom', now, rospy.Duration(1))
        # transGoal = self._odom_list.transformPose('base_link', goal)
        #
        # (goal_angle_roll, goal_angle_pitch, goal_angle_yaw) = tf.transformations.euler_from_quaternion(
        #     [transGoal.pose.orientation.x,
        #      transGoal.pose.orientation.y,
        #      transGoal.pose.orientation.z,
        #      transGoal.pose.orientation.w])

    def drive_straight(self, speed, distance):
        """
        Make the turtlebot drive straight
        :type speed: float
        :type distance: float
        :param speed: speed to drive
        :param distance: distance to drive
        :return:
        """

        drive_rate = rospy.Rate(10)  # 10hz
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        start_px = self.px
        start_py = self.py
        dist_traveled=math.sqrt(math.pow(start_px-self.px, 2) + math.pow(start_py-self.py, 2))

        while dist_traveled < distance:  # and move it forward
            dist_traveled = math.sqrt(math.pow(start_px - self.px, 2) + math.pow(start_py - self.py, 2))
            self.vel_msg.linear.x = speed
            self.vel_pub.publish(self.vel_msg)
            drive_rate.sleep()

        self.vel_msg.linear.x = 0
        self.vel_pub.publish(self.vel_msg)

    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        rospy.loginfo("my roatation is {}".format(self.zRot-angle))
        while (self.zRot-angle) > .01:
            self.vel_msg.angular.z = -1
            self.vel_pub.publish(self.vel_msg)
            #rospy.loginfo("my roatation is {}".format(self.zRot-angle))

        while (self.zRot-angle) < -.01:
            self.vel_msg.angular.z = 1
            self.vel_pub.publish(self.vel_msg)
            #rospy.loginfo("my roatation is {}".format(self.zRot-angle))

        rospy.loginfo("exited loop at {}".format(self.zRot - angle))
        self.vel_msg.angular.z = 0
        self.vel_pub.publish(self.vel_msg)


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
       # rospy.loginfo("-----------------------------")
        # rospy.loginfo("hello from odom")
        # rospy.loginfo("my roll is {}".format(self.px))


if __name__ == '__main__':
    rospy.init_node('Robot')
    ne = Robot()
    # ne.drive_straight(.2, 2)
    # ne.rotate(-math.pi)
    # ne.rotate(0)
    while not rospy.is_shutdown():
        rate = rospy.Rate(10)  # 10hz
        rate.sleep()
    pass
