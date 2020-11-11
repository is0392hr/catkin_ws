#!/bin/sh
rosservice call /drone07/enable_motors true
sleep 1
rosrun multi_robot_unity test_d7.py
