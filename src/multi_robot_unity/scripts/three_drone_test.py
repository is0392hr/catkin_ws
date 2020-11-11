
import rospy
from sample_py.msg import sample_message
from geometry_msgs.msg import Twist, Point
import tf
from math import pow, sqrt, pi, radians
import PyKDL

class TurtleBotSim():
    def __init__(self):
        rospy.init_node('robot_state_publisher', anonymous=True)
        rospy.loginfo(" Press CTRL+c to stop Drones")
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel1 = rospy.Publisher('drone1/cmd_vel', Twist, queue_size=10)
        self.cmd_vel2 = rospy.Publisher('drone2/cmd_vel', Twist, queue_size=10)
	self.cmd_vel3 = rospy.Publisher('drone3/cmd_vel', Twist, queue_size=10)
        r = rospy.Rate(10);
        twist = Twist()
	twist.linear.x = 0
	twist.linear.y = 0
        twist.linear.z = 0.1
        twist.angular.x = 0
	twist.angular.y = 0
        twist.angular.z = 0


        while not rospy.is_shutdown():
	    self.cmd_vel1.publish(twist)
            self.cmd_vel2.publish(twist)
	    self.cmd_vel3.publish(twist)
            r.sleep()


    def shutdown(self):
        rospy.loginfo("Stop TurtleBot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        TurtleBotSim()
    except:
        rospy.loginfo("Two Drones finish")
