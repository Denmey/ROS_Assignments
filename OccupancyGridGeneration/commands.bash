#!/bin/bash

# cd src
# catkin_init_workspace
# cd ..

export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2; exit;}'):0 

catkin_make
source ./devel/setup.bash
chmod +x ./src/occupancy_grid/scripts/main.py

rosbag play mit_1.bag &
roslaunch occupancy_grid run.launch

# kill %1 
