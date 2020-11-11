#!/bin/sh
rosservice call /drone15/enable_motors true
sleep 1
rosrun multi_robot_unity test_d15.py
