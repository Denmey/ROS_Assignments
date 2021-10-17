#!/bin/bash

# cd src
# catkin_init_workspace
# cd ..

catkin_make
source ./devel/setup.bash

sleep_time=5

roscore &
sleep $sleep_time

rosrun turtlesim turtlesim_node &
sleep $sleep_time
rosservice call /kill "name: 'turtle1'"
rosservice call /spawn "{x: 0.0, y: 0.0, theta: 1.5707, name: 'chaser'}"
rosservice call /spawn "{x: 3.0, y: 6.0, theta: 0.0, name: 'chased'}"
rqt_graph &
sleep $sleep_time

chmod +x ./src/chased/scripts/chased.py
rosrun chased chased.py &

chmod +x ./src/chaser/scripts/chaser.py
rosrun chaser chaser.py &

read -p "Press enter to close everything"

kill %5
kill %4
kill %3
kill %2
kill %1
