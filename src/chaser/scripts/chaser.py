#!/usr/bin/python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt, pi

# Common constants
RATE = 10

# Chaser-specific constants


chaser_pose = None
chased_pose = None


def distance(pos1, pos2):
    return sqrt(pow((pos1.x - pos2.x), 2) + pow((pos1.y - pos2.y), 2))
def rotate_from_to(rot_current, rot_goal):
    delta = rot_goal - rot_current
    while delta > pi:
        delta -= 2*pi
    while delta < -pi:
        delta += 2*pi
    return delta
def steering_angle(pos, goal_pos):
    return atan2(goal_pos.y - pos.y, goal_pos.x - pos.x)
def angular_vel(pos, goal_pos):
    return 4 * (steering_angle(pos, goal_pos) - pos.theta)

def chaser_pose_callback(msg):
    global chaser_pose
    chaser_pose = msg

def chased_pose_callback(msg):
    global chased_pose
    chased_pose = msg
    # print(msg)

rospy.init_node('chaser')

rospy.Subscriber('/chaser/pose', Pose, chaser_pose_callback)
rospy.Subscriber('/chased/pose', Pose, chased_pose_callback)
chaser_velocity_publisher = rospy.Publisher('/chaser/cmd_vel', Twist, queue_size = 1)


rate = rospy.Rate(RATE)

while not rospy.is_shutdown():
    if chaser_pose:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        if chased_pose and distance(chaser_pose, chased_pose) > 0.5:
            # msg.linear.x = 2.85
            msg.linear.x = 0.98*4.0

            target_angle = atan2(chased_pose.y - chaser_pose.y, chased_pose.x - chaser_pose.x)
            msg.angular.z = rotate_from_to(chaser_pose.theta, target_angle) * RATE
            # msg.angular.z = angular_vel(chaser_pose, chased_pose)

        chaser_velocity_publisher.publish(msg)

    rate.sleep()