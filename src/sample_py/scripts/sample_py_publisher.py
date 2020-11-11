#!/usr/bin/env python
# license removed for brevity

import rospy
from sample_py.msg import sample_message

def publisher():
    rospy.init_node('sample_py_publisher', anonymous=True)
    pub = rospy.Publisher('sample_topic', sample_message, queue_size=10)
    rate = rospy.Rate(2)

    count = 0
    while not rospy.is_shutdown():
        str = "hello world"
        rospy.loginfo("message = %s, count = %d" %(str, count))
        msg = sample_message()
        msg.message = str
        msg.count = count

        pub.publish(msg)
        rate.sleep()
        count += 1


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException: pass
