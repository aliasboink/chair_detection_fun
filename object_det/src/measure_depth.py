#!/usr/bin/python2
import rospy
import cv2
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge 
import numpy as np

''' 
Raspberry PI 640
projection
503.704010 0.000000 329.486994 0.000000
0.000000 505.431824 238.680651 0.000000
0.000000 0.000000 1.000000 0.000000
'''
fx = 503.704010
fy = 505.431824
cx = 329.486994
cy = 238.680561
chair_height = 75 # centimeters, the previous values are pixels
marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)


def det_callback(det_data):
    #rospy.loginfo("Received data...")
    #rospy.loginfo(det_data.detections[0])
    # Choose only chairs.
    for elem in [x for x in det_data.detections if x.results[0].id == 62]:
        depth = (fy/elem.bbox.size_y)*chair_height 
        x = (elem.bbox.center.x - cx) * (depth/fx)
        y = (elem.bbox.center.y - cy) * (depth/fy)
        rospy.loginfo(depth)
        rospy.loginfo("x: " + str(x))
        rospy.loginfo("y: " + str(y))

        # Create a marker for the chair 
        marker = Marker()
        # This should be probably changed to "/base_link".
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = depth
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2
        marker.scale.y = 2
        marker.scale.z = 2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker_pub.publish(marker)
:orangu

def main():
    rospy.init_node("chair_calculator", anonymous=True)
    rospy.Subscriber("/detectnet/detections", Detection2DArray, det_callback)
    #marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.spin()

if __name__== '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass