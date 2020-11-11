#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
import numpy as np


class Centroid():
    def __init__(self):
        self.data = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.pub = rospy.Publisher('Centroid', Float64MultiArray, queue_size=10)
    def publisher():
      rospy.init_node('Key', anonymous=True)
      pub = self.pub

      while not rospy.is_shutdown():
        key = input()
        print(key)
        pub.publish(key)
        

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

if __name__ == '__main__':
    controler = Controler()
    controler.publisher()