#!/usr/bin/python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


MAX_FLOAT = 5000.0
# Occupancy grid parameters
GRID_WIDTH_UNITS = 30
GRID_HEIGHT_UNITS = 30
GRID_RESOLUTION = 0.1
GRID_WIDTH_CELLS = int(GRID_WIDTH_UNITS / GRID_RESOLUTION)
GRID_HEIGHT_CELLS = int(GRID_HEIGHT_UNITS / GRID_RESOLUTION)

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

def init_occupancy_grid_msg():
    occupancy_grid_msg = OccupancyGrid()

    occupancy_grid_msg.header.frame_id = "base_link"
    occupancy_grid_msg.info.width = GRID_WIDTH_CELLS
    occupancy_grid_msg.info.height = GRID_HEIGHT_CELLS
    occupancy_grid_msg.info.resolution = GRID_RESOLUTION
    occupancy_grid_msg.info.origin.position.x = -GRID_WIDTH_UNITS/2.0
    occupancy_grid_msg.info.origin.position.y = -GRID_HEIGHT_UNITS/2.0
    occupancy_grid_msg.info.origin.position.z = 0.25
    occupancy_grid_msg.data = [-1] * (GRID_WIDTH_CELLS * GRID_HEIGHT_CELLS)

    return occupancy_grid_msg

def get_cell(x, y):
    # Origin of offset is unknown :(
    OFFSET_X = 0.25
    OFFSET_Y = -0.025

    cell_x = round((x + OFFSET_X + GRID_WIDTH_UNITS/2) / GRID_RESOLUTION) # Translate from [-s/2, s/2] to [0, s]
    cell_y = round((y + OFFSET_Y + GRID_HEIGHT_UNITS/2) / GRID_RESOLUTION)

    return cell_x, cell_y

# Sets -1 cells to 0 along the ray
def cast_ray_and_set_free_cells(grid, dist, theta):
    STEP = GRID_RESOLUTION / 2.0
    STEPS = dist / STEP
    for i in range(0, int(STEPS)):
        r = dist / STEPS * i
        x, y = polar2cart(r, theta)
        cell_x, cell_y = get_cell(x, y)
        index = cell_x + cell_y * GRID_WIDTH_CELLS
        if grid[index] == -1:
            grid[index] = 0

def get_occupancy_grid_msg(ranges, angle_min, angle_step):

    occupancy_grid_msg = init_occupancy_grid_msg()
    
    # occupancy_grid_msg.data[0] = 0
    # occupancy_grid_msg.data[1] = 0

    for i, distance in enumerate(ranges):
        if distance > MAX_FLOAT-1.0:
            continue
        angle = angle_min+angle_step*i

        # Performance heavy thing
        cast_ray_and_set_free_cells(occupancy_grid_msg.data, distance, angle)

        coords = polar2cart(distance, angle)
        cell_x, cell_y = get_cell(coords[0], coords[1])
        occupancy_grid_msg.data[cell_x + cell_y * GRID_WIDTH_CELLS] = 100

    return occupancy_grid_msg

def process_scan(msg):
    original_publisher.publish(msg)

    msg.ranges = remove_outliers(msg.ranges, msg.angle_min, msg.angle_increment)
    no_outliers_publisher.publish(msg)

    occupancy_grid_publisher.publish(get_occupancy_grid_msg(msg.ranges, msg.angle_min, msg.angle_increment))

rospy.Subscriber('base_scan', LaserScan, process_scan)
print("Script initialization done")

while not rospy.is_shutdown():
    rate.sleep()