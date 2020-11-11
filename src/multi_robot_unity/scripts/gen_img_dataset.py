#!/usr/bin/env python3

import rospy
import os 

import cv2, cv_bridge
from sensor_msgs.msg import Image, CameraInfo, CompressedImage

from segmentation import predict

model = predict.model_from_checkpoint_path("checkpoints/")

class Follower:
    def __init__(self):
    
        self.bridge = cv_bridge.CvBridge()
        image = rospy.Subscriber('/unity_image/compressed_main', CompressedImage, self.image_callback)     

    def image_callback(self, msg):

        image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8') #bgr8

        global model
        #cv2.imshow("Main", image)
        #cv2.waitKey(3)

        seg_img = predict.action(model, image)
        seg_img = cv2.resize(seg_img, (1920, 1080))
        #cv2.imwrite("dataset/"+str(datetime.now())+'.jpg', image)
        cv2.imshow("Main", seg_img)
        cv2.waitKey(3)
    

    
    
def listener():
    rospy.init_node('gen_img_dataset')
    follower = Follower()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
# END ALL
