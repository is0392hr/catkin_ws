#!/usr/bin/python
import rospy
from sample_py.msg import sample_message


def callback(msg):
    rospy.loginfo("I heard: message = [%s], count = [%d]" % (msg.message, msg.count));


def subscriber():
    rospy.init_node('sample_py_subscriber', anonymous=True)
    rospy.Subscriber('sample_topic', sample_message, callback)

    rospy.spin()

if __name__ == '__main__':
    subscriber()
