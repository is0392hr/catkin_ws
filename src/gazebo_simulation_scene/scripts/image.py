#! /usr/bin/env python

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import CompressedImage
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2, cv_bridge


class Camera:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		image = rospy.Subscriber('/unity_image/compressed', CompressedImage, self.image_callback)
		
	def image_callback(self, msg):
		
		image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
		cv2.imshow('camera', image)
		cv2.waitKey(3)

# Instantiate CvBridge
#bridge = CvBridge()


def main():
   	rospy.init_node('camera_jpg')

	camera = Camera()

	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
