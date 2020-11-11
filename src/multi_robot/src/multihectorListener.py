import os
import sys
import tempfile
import csv
import tf
import rospy
# Messages
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

filename = "dataExchangeROS2Math.csv"

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + '%s', data)
    names = data.name
    poseDictBots = {data.name[i] : data.pose[i] for i in range(nBots+1)}
    twistDictBots = {data.name[i] : data.twist[i] for i in range(nBots+1)}

#rospy.loginfo('%s', data.pose.pose.position.x)
    with tempfile.NamedTemporaryFile('w', dir=os.path.dirname(filename),delete=False) as fake_file:
        with fake_file as fake_csv:
            writer = csv.writer(fake_csv, delimiter=',')
	    for i in range(1,nBots+1):
		orienX = poseDictBots["Robot"+str(i)].orientation.x
		orienY = poseDictBots["Robot"+str(i)].orientation.y
		orienZ = poseDictBots["Robot"+str(i)].orientation.z
		orienW = poseDictBots["Robot"+str(i)].orientation.w

		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orienX, orienY, orienZ,orienW])
		velLin = twistDictBots["Robot"+str(i)].linear
		velAngular = twistDictBots["Robot"+str(i)].angular
		posX = poseDictBots["Robot"+str(i)].position.x
		posY = poseDictBots["Robot"+str(i)].position.y
		posZ = poseDictBots["Robot"+str(i)].position.z
		odometryTuple = [posX,posY,posZ,velLin.x,velLin.y,velLin.z,velAngular.x,velAngular.y,velAngular.z,yaw]
		writer.writerow(odometryTuple)
	    tempFileName = fake_file.name
	os.rename(tempFileName, filename)

def listener():
# In ROS, nodes are uniquely named. If two nodes with the same# name are launched, the previous one is kicked off. The# anonymous=True flag means that rospy will choose a unique# name for our listener node so that multiple listeners can# run simultaneously.
    rospy.init_node('listener', anonymous=True)

    #Reading the position directly from Gazebo
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback)

    # Use the following piece to get the position from Odometry. Notethat it is not very accurate.
    #rospy.Subscriber('/odom', Odometry, callbackOdometry)
    rospy.spin()

if __name__ == '__main__':
    try:
        nBots = int(sys.argv[1])
    except ValueError:
	print "The 'nBots' parameteter is required. \n Try '***.py nBots'"
	sys.exit()
    listener()
