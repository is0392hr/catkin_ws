import rospy
from sample_py.msg import sample_message
from geometry_msgs.msg import Twist, Point, PoseStamped
import tf
from math import pow, sqrt, pi, radians
import PyKDL
from nav_msgs.msg import Odometry

def position(msg):
    print(msg.pose.position)

if __name__ == '__main__':
    try:     
        rospy.init_node('odometry', anonymous=True)
        rospy.Subscriber('drone/ground_truth_to_tf/pose', PoseStamped, position)
        rospy.spin()

    except:
        rospy.loginfo("TurtleBot finish")

