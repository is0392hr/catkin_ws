#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from pbl_turtlebot.msg import sample_message


def callback(msg):
    rospy.loginfo("I heard: message = [%s], count = [%d]" % (msg.message, msg.count));


def subscriber():
  
    rospy.init_node('subscriber', anonymous=True)
    
    rospy.Subscriber('sample_topic', sample_message, callback)

    rospy.spin()

if __name__ == '__main__':
    subscriber()
