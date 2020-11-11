import rospy
from sample_py.msg import sample_message
from geometry_msgs.msg import Twist, Point, PoseStamped
import tf
from math import pow, sqrt, pi, radians
import PyKDL
from nav_msgs.msg import Odometry

cmd_vel = rospy.Publisher('drone/cmd_vel', Twist, queue_size=10)


def calc(msg, num):
    if num == 0:
        x_t = msg.pose.pose.position.x
        y_t = msg.pose.pose.position.y
        z_t = msg.pose.pose.position.z
    if num == 1:
        x_d = msg_drone.pose.position.x
        y_d = msg_drone.pose.position.y
        z_d = msg_drone.pose.position.z
	print(x_t)
	twist = Twist()
	twist.linear.x = x_t - x_d
	twist.linear.y = y_t - y_d
	twist.linear.z = 0.01
	cmd_vel.publish(twist)
	

def DroneSim():
    rospy.init_node('controller', anonymous=True)
    rospy.loginfo(" Press CTRL+c to stop Drone")
    rospy.Subscriber('tb3/odom', Odometry, calc, arg=0)
    rospy.Subscriber('drone/ground_truth_to_tf/pose', calc, arg=1)
    rospy.spin()
    
    
if __name__ == '__main__':
    try:
        DroneSim()
        
    except:
        rospy.loginfo("Drone finish")
