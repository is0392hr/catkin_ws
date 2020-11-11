
import rospy
from sample_py.msg import sample_message
from geometry_msgs.msg import Twist, Point
import tf
from math import pow, sqrt, pi, radians
import PyKDL
from nav_msgs.msg import Odometry


cmd_vel = rospy.Publisher('drone/cmd_vel', Twist, queue_size=10)

def odometryCb(msg):
    x = msg.twist.twist.linear.x
    y = msg.twist.twist.linear.y
    az = msg.twist.twist.angular.z
    twist = Twist()
    twist.linear.x = x
    twist.linear.y = y
    twist.linear.z = 0.01
    twist.angular.z = az
    cmd_vel.publish(twist)
    rospy.Rate(10).sleep
    
def DroneSim():
    rospy.init_node('odometry', anonymous=True)
    rospy.loginfo(" Press CTRL+c to stop Drone")
    rospy.Subscriber('tb3/odom', Odometry, odometryCb)
    rospy.spin()
    
    
if __name__ == '__main__':
    try:
        DroneSim()
        
    except:
        rospy.loginfo("Drone finish")
