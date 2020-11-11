#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from pbl_turtlebot.msg import SensorState

class Sonar():
    def __init__(self):
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.sonar_sub = rospy.Subscriber('sensor_state', SensorState, self.get_sonar, queue_size = 1)
        self.sonar()

    def get_sonar(self, sensor):
        twist = Twist()
        if sensor.sonar < 10:
            linear_vel = 0
        else:
            linear_vel = 0.05

        twist.linear.x = linear_vel
        self.cmd_pub.publish(twist)

    def sonar(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

def main():
    rospy.init_node('turtlebot3_sonar')
    try:
        sonar = Sonar()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
