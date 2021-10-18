#!/usr/bin/python3

import rospy
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

ARROW_MARKER        = 0
CUBE_MARKER         = 1
SPHERE_MARKER       = 2
CYLINDER_MARKER     = 3
LINE_STRIP_MARKER   = 4
LINE_LIST_MARKER    = 5
CUBE_LIST_MARKER    = 6
SPHERE_LIST_MARKER  = 7
POINTS_MARKER       = 8
# ...


# rospy.init_node('rviz_marker')
# marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
# marker = Marker()
# marker.header.frame_id = "world"
# # marker.header.stamp = rospy.Time.now()
# marker.type   = POINTS_MARKER
# marker.id     = 0
# marker.action = 0
# # Set the scale of the marker
# marker.scale.x = 1.0
# marker.scale.y = 1.0
# marker.scale.z = 1.0
# # Set the color
# marker.color.r = 0.0
# marker.color.g = 1.0
# marker.color.b = 0.0
# marker.color.a = 1.0
# # Set the pose of the marker
# marker.pose.position.x = 0
# marker.pose.position.y = 0
# marker.pose.position.z = 0
# # marker.pose.orientation.x = 0.0
# # marker.pose.orientation.y = 0.0
# # marker.pose.orientation.z = 0.0
# # marker.pose.orientation.w = 1.0
# marker.points.append(Point(8,8,0))
# rate = rospy.Rate(1)
# # while not rospy.is_shutdown():
# rate.sleep()
# marker_pub.publish(marker)

rospy.init_node('occupancy_grid_generator')
rate = rospy.Rate(1)

# original_publisher = rospy.Publisher('/base_scan_original', LaserScan, queue_size = 10)
no_outliers_publisher = rospy.Publisher('/base_scan_no_outliers', LaserScan, queue_size = 10)

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

# Points are in polar coords
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
    # original_publisher.publish(msg)

    msg.ranges = remove_outliers(msg.ranges, msg.angle_min, msg.angle_increment)
    no_outliers_publisher.publish(msg)

rospy.Subscriber('base_scan', LaserScan, process_scan)
print("Script initialization done")

while not rospy.is_shutdown():
    rate.sleep()