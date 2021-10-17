#!/usr/bin/python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt, pi
import random

# Common constants
RATE = 10

# Chased-specific params
RUN_FROM_WALLS_THRESHOLD = 1.0
RUN_FROM_WALLS_FACTOR = 100.0
RUN_FROM_CHASER_DIST_THRESHOLD = 3.0
RUN_FROM_CHASER_FACTOR = 3.0
RUN_ALONG_WALL_THRESHOLD = 3.0 # Distance from room center to begin running along the wall in counter clockwise direction
RUN_ALONG_WALL_FACTOR = 10.0
RANDOMNESS_RANGE = 25.0 # Max abs random angle in degrees

chaser_pose = None
chased_pose = None

def length(x, y):
    return sqrt(pow(x, 2) + pow(y, 2))    
def distance(pos1, pos2):
    return sqrt(pow((pos1.x - pos2.x), 2) + pow((pos1.y - pos2.y), 2))
def distance_to_walls(pos):
    return min(pos.x, abs(11.2-pos.x), pos.y, abs(11.2-pos.y))
def run_along_wall_angle(pos):
    x = pos.x - 5.6
    y = pos.y - 5.6
    nx = -y
    ny = x
    return atan2(ny, nx)
# Angles need to be between 0 and 2pi
def rotate_from_to(rot_current, rot_goal):
    delta = rot_goal - rot_current
    while delta > pi:
        delta -= 2*pi
    while delta < -pi:
        delta += 2*pi
    return delta

def chaser_pose_callback(msg):
    global chaser_pose
    chaser_pose = msg

def chased_pose_callback(msg):
    global chased_pose
    chased_pose = msg

rospy.init_node('chased')

rospy.Subscriber('/chaser/pose', Pose, chaser_pose_callback)
rospy.Subscriber('/chased/pose', Pose, chased_pose_callback)
chased_velocity_publisher = rospy.Publisher('/chased/cmd_vel', Twist, queue_size = 1)

rate = rospy.Rate(RATE)
i = 0
randomness_angle = (random.random() - 0.5) * 2.0 * RANDOMNESS_RANGE / 180.0 * pi
while not rospy.is_shutdown():
    i += 1
    if chased_pose:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        # Running from walls
        running_from_walls = False
        run_from_walls_dir_x = (5.6 - chased_pose.x)
        run_from_walls_dir_y = (5.6 - chased_pose.y)
        run_from_walls_angle = 0.0
        if distance_to_walls(chased_pose) < RUN_FROM_WALLS_THRESHOLD:
            running_from_walls = True
            run_from_walls_angle = atan2(run_from_walls_dir_y, run_from_walls_dir_x)
        
        # Running from chaser
        run_from_chaser_angle = 0.0
        dist_to_chaser = 0.0
        running_from_chaser = False
        if chaser_pose:
            dist_to_chaser = distance(chased_pose, chaser_pose)
            if dist_to_chaser < RUN_FROM_CHASER_DIST_THRESHOLD:
                running_from_chaser = True
                run_from_chaser_angle = atan2( chased_pose.y-chaser_pose.y, chased_pose.x-chaser_pose.x )
        
        # Run along the wall
        running_along_wall = False
        if length(chased_pose.x-5.6, chased_pose.y-5.6) > RUN_ALONG_WALL_THRESHOLD:
            running_along_wall = True

        running_along_wall = False
        msg.linear.x = 5.0

        denominator = running_from_walls * RUN_FROM_WALLS_FACTOR + running_from_chaser * RUN_FROM_CHASER_FACTOR + running_along_wall * RUN_ALONG_WALL_FACTOR
        numerator = running_from_walls * RUN_FROM_WALLS_FACTOR * run_from_walls_angle 
        numerator += running_from_chaser * RUN_FROM_CHASER_FACTOR * run_from_chaser_angle
        numerator += running_along_wall * RUN_ALONG_WALL_FACTOR * run_along_wall_angle(chased_pose)

        if i == RATE:
            randomness_angle = (random.random() - 0.5) * 2.0 * RANDOMNESS_RANGE / 180.0 * pi
            i = 0


        if denominator > 0.005:
            target_angle = numerator / denominator + randomness_angle
            # msg.linear.x = 3.0
            msg.angular.z = rotate_from_to(chased_pose.theta, target_angle) * RATE

        chased_velocity_publisher.publish(msg)

    rate.sleep()