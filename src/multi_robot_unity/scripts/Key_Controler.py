#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np


class Controler():
    def __init__(self):
        self.pub = rospy.Publisher('key', String, queue_size=10)
     
    def publisher(self):
      
      pub = self.pub

      while not rospy.is_shutdown():
        key = raw_input()
        print(key)
        pub.publish(key)
        rospy.Rate(1).sleep

if __name__ == '__main__':
  rospy.init_node('key', anonymous=False)
  controler = Controler()
  controler.publisher()
    