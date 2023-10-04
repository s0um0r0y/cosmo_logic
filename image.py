import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
# from calibirate_position import calibirator
import json

class Object_detector:


    def callback(self,data):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data,"bgr8")
        cv2.namedWindow('Live Video')
        cv2.imshow('Live Video',image)
        cv2.waitKey(1)
        

    def listener(self):
        rospy.init_node('image_viewer',anonymous= True)
        
        rospy.Subscriber('/camera1/image_raw',Image, self.callback)
        
        rospy.spin()



# Testing and current Running
    
if __name__ == '__main__':
    x = Object_detector()    
    x.listener()
