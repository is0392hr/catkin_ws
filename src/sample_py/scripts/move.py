#!/usr/bin/env python
# license removed for brevity

import rospy
# headerfile for generated message
from sample_py.msg import sample_message
from geometry_msgs.msg import Twist, Point
import tf
from math import pow, sqrt, pi, radians
import PyKDL

def publisher(): 
    while(True):
        start_trans, start_rot = get_odom()
        print "Option:'l' for linear operation"
        print "       'a' for rotation"
        print "       'p' for goal position"
        y = raw_input("Enter mode:")
        print y
        if y == 'l':
            speed = float(input("Speed:"))
            dist = float(input("Dist:")) * 100
            linear(speed,dist,start_trans)
            
        elif y == 'a':
            w = float(input("Angualr vel:"))
            angle = float(input("Angle:"))
            rad = radians(angle)
            rotate(w, rad, True)
            
        elif y == 'p':
            x = float(input("Goal position X:"))
            y = float(input("Goal position Y:"))
            speed = float(input("Speed:"))
            dist = float(input("Dist:")) * 100
            w = float(input("Angualr vel:"))
            angle = float(input("Angle:"))
            rad = radians(angle)
            position(start_trans, start_rot, speed, dist, w, rad)
            
def get_odom():
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform('odom', 'base_footprint', rospy.Time(), rospy.Duration(1.0))
    try:
        (trans, rot) =tf_listener.lookupTransform('odom','base_footprint', rospy.Time(0))

    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return
    return Point(*trans), quat_to_angle(rot)

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3])
    return rot.GetRPY()[2]

def norm_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while  res < -pi:
        res += 2.0 * pi
    return res

def linear(speed, dist, start_trans):
    cmd_msg = Twist()
    cmd_msg.linear.x = speed
    pub.publish(cmd_msg)
    current_dist = 0
    while current_dist < dist:
        trans,rot = get_odom()
        print trans, rot
        current_dist += sqrt(pow((trans.x - start_trans.x),2) + pow((trans.y - start_trans.y),2))

    cmd_msg.linear.x = 0
    pub.publish(cmd_msg)
          

def rotate(w, rad, start_rot):
    tolerance_rad = radians(2.5)
    cmd_msg = Twist()
    cmd_msg.angular.z = w
    pub.publish(cmd_msg)
    current_rad = 0.0
    last_rad = start_rot
    while (abs(current_rad + tolerance_rad)  < abs(rad)):
        tras, rot = get_odom()
        print rot
        rad_delta = norm_angle(rot - last_rad)
        current_rad += rad_delta
        last_rad = rot
        
    cmd_msg.angular.z = 0
    pub.publish(cmd_msg)

def position(start_trans,start_rod, speed, dist, w, angle, x, y):
    tolerance_rad = radians(2.5)
    cmd_msg = Twist()
    cmd_msg.linear.x = speed
    cmd_msg.angular.z = w
    
    
if __name__ == '__main__':
    try:
        # init node and name "pbl_publisher"
        rospy.init_node('robot_state_publisher', anonymous=True)
        # send message sample_message to topic "sample_topic"
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # send 2 data/s
        rate = rospy.Rate(2)
        publisher()
    except rospy.ROSInterruptException: pass
