#!/bin/bash --init-file
xterm -T "ROS-Core" -e "roscore" &
xterm -T "RQT" -e "rqt" &
xterm -T "Terminal"