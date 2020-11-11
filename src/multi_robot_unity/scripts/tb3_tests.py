
import rospy
from sample_py.msg import sample_message
from geometry_msgs.msg import Twist, Point
import tf
from math import pow, sqrt, pi, radians
import PyKDL

class TurtleBotSim():
    def __init__(self):
        rospy.init_node('robot_state_publisher', anonymous=True)


        rospy.loginfo(" Press CTRL+c to stop TurtleBot")

        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('tb3/cmd_vel', Twist, queue_size=10)

        r = rospy.Rate(10);
        twist = Twist()

        twist.linear.x = -1
        twist.angular.z = 0.3

        while not rospy.is_shutdown():
            self.cmd_vel.publish(twist)
            r.sleep()


    def shutdown(self):
        rospy.loginfo("Stop TurtleBot")
        self.cmd_vel.publish(Twist())
        r.sleep()

if __name__ == '__main__':
    try:
        TurtleBotSim()
    except:
        rospy.loginfo("TurtleBot finish")

