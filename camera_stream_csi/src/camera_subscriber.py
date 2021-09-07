#!/usr/bin/python2
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import numpy as np

br = CvBridge()

def img_callback(image_data):
    rospy.loginfo("Received frame...")
    current_frame = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
    #current_frame = br.imgmsg_to_cv2(data)
    cv2.imshow("camera", current_frame)
    cv2.waitKey(1)

def main():
    rospy.init_node('video_receiver')
    rospy.Subscriber('/detectnet/overlay', Image, img_callback)
    rospy.spin()

if __name__== '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass