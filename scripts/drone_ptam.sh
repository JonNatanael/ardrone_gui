#!/bin/bash

roscore &
sleep 2
rqt &
ROS_NAMESPACE=ardrone rosrun image_proc image_proc &
roslaunch ptam ptam.launch &
roslaunch ardrone_autonomy ardrone.launch


trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
ps -A | grep ros
ps -A | grep ardrone_driver