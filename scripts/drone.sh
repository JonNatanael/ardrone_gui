#!/bin/bash

roscore &
sleep 2
rqt &
roslaunch ardrone_autonomy ardrone.launch


trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
ps -A | grep ros
ps -A | grep ardrone_driver