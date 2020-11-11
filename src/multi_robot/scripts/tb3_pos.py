
import rospy

from nav_msgs.msg import Odometry


def odometryCb(msg):
    print(msg.pose.pose.position)

if __name__ == '__main__':
    try:     
        rospy.init_node('odometry', anonymous=True)
        rospy.Subscriber('tb3/odom', Odometry, odometryCb)
        rospy.spin()

    except:
        rospy.loginfo("TurtleBot finish")

