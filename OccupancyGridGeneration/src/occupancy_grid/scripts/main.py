#!/usr/bin/python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

# Occupancy grid parameters
GRID_WIDTH = 30
GRID_HEIGHT = 30
GRID_RESOLUTION = 0.1


rospy.init_node('occupancy_grid_generator')
rate = rospy.Rate(1)

original_publisher = rospy.Publisher('/base_scan_original', LaserScan, queue_size = 10)
no_outliers_publisher = rospy.Publisher('/base_scan_no_outliers', LaserScan, queue_size = 10)
occupancy_grid_publisher = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size = 10)

def polar2cart(r, theta):
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return x,y
def dist2(p1, p2):
    sum = 0.0
    for v1, v2 in zip(p1, p2):
        sum += (v1 - v2) ** 2
    return sum
def dist(p1, p2):
    return math.sqrt(dist2(p1, p2))

def remove_outliers(ranges, angle_min, angle_step):
    OVERSTEP = 3
    MAX_FLOAT = 5000.0
    MAX_DISTANCE = 0.05
    results = [MAX_FLOAT] * OVERSTEP

    for i in range(OVERSTEP, len(ranges)-OVERSTEP):
        angle_l = angle_min + angle_step * (i+OVERSTEP)
        angle   = angle_min + angle_step * i
        angle_r = angle_min + angle_step * (i-OVERSTEP)

        pos_l = polar2cart(ranges[i+OVERSTEP], angle_l)
        pos   = polar2cart(ranges[i],   angle  )
        pos_r = polar2cart(ranges[i-OVERSTEP], angle_r)

        # print(dist(pos_l, pos) + dist(pos_r, pos))
        if (dist(pos_l, pos) + dist(pos_r, pos)) < MAX_DISTANCE*OVERSTEP*2.0:
            results.append(ranges[i])
        else:
            results.append(MAX_FLOAT)

    results.extend([MAX_FLOAT] * OVERSTEP)

    return results


def process_scan(msg):
    original_publisher.publish(msg)

    msg.ranges = remove_outliers(msg.ranges, msg.angle_min, msg.angle_increment)
    no_outliers_publisher.publish(msg)

    occupancy_grid_msg = OccupancyGrid()


    width = int(GRID_WIDTH / GRID_RESOLUTION)
    height = int(GRID_HEIGHT / GRID_RESOLUTION)
    occupancy_grid_msg.header.frame_id = "base_link"
    occupancy_grid_msg.info.width = width
    occupancy_grid_msg.info.height = height
    occupancy_grid_msg.info.resolution = GRID_RESOLUTION
    occupancy_grid_msg.info.origin.position.x = -GRID_WIDTH/2.0
    occupancy_grid_msg.info.origin.position.y = -GRID_HEIGHT/2.0
    occupancy_grid_msg.data = [-1] * (width * height)

    occupancy_grid_publisher.publish(occupancy_grid_msg)

rospy.Subscriber('base_scan', LaserScan, process_scan)
print("Script initialization done")

while not rospy.is_shutdown():
    rate.sleep()