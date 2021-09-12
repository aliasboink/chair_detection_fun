#!/usr/bin/python2
import rospy
import cv2
import tf2_ros
import geometry_msgs.msg
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge 
import numpy as np

#Frankly, I should've just made a class for all of this, as one tends to do.
#Yeah, it's getting far too messy.
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
chair_height = 75 #centimeters, the previous values are pixels
# ROS variables
rospy.init_node("chair_calculator", anonymous=True)
rate = rospy.Rate(10.0) 
tf_buffer = tf2_ros.Buffer() #tf2 transform buffer
tf_listener = tf2_ros.TransformListener(tf_buffer)#tf2 listener to get the frame transforms
marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10) #Marker (RViz) publisher
detect_pub = rospy.Publisher("detect_orb_data", Float32MultiArray, queue_size=10) #Publisher for ORB-SLAM2 data, 5 float variables.


def det_callback(det_data):
    #rospy.loginfo("Received data...")
    #rospy.loginfo(det_data.detections[0])
    #Choose only chairs.
    for elem in [x for x in det_data.detections if x.results[0].id == 62 and x.results[0].score >= 0.95]:
        depth = (fy/elem.bbox.size_y)*chair_height/10 # 10 for the cm to decimeters conversion. <3 Testing something.
        x = (elem.bbox.center.x - cx) * (depth/fx)
        y = (elem.bbox.center.y - cy) * (depth/fy)
        rospy.loginfo(depth)
        rospy.loginfo("x: " + str(x))
        rospy.loginfo("y: " + str(y))
        rospy.loginfo("score: " + str(elem.results[0].score))

        #Send data to modified ORB-SLAM2
        float_data = Float32MultiArray()
        float_data.data = [depth, elem.bbox.center.x, elem.bbox.center.y, elem.bbox.size_y, elem.bbox.size_x]
        detect_pub.publish(float_data)

        # Fetch the transformation from the camera's frame to the map's frame.
        try:
            trans = tf_buffer.lookup_transform("base_link", "map", rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        #rospy.loginfo(trans)

        # Create a marker for the chair 
        marker = Marker()
        # This should be probably changed to "/base_link".
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = depth - trans.transform.translation.x
        marker.pose.position.y = -x - trans.transform.translation.y
        marker.pose.position.z = -y - trans.transform.translation.z
        rospy.loginfo("y trans: " + str(trans.transform.translation.y))
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.5
        marker.scale.y = 1.5    
        marker.scale.z = 1.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.45
        marker.color.b = 0.8
        marker_pub.publish(marker)
        rate.sleep()

def main():
    rospy.Subscriber("/detectnet/detections", Detection2DArray, det_callback, queue_size=2)
    #marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.spin()

if __name__== '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
