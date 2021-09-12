#!/usr/bin/python2
import rospy
import cv2
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
import geometry_msgs.msg
from threading import Thread
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge 
import numpy as np

''' 
Raspberry PI 640
projection
503.704010 0.000000 329.486994 0.000000
0.000000 505.431824 238.680651 0.000000
0.000000 0.000000 1.000000 0.000000
'''
class ChairMagic():

    def __init__(self):
        #super(ChairMagic, self).__init__()
        #self.daemon = True
        #self._running = True
        # ===================== CAMERA ELEMENTS ======================= #
        # (and the chair height)
        self.fx = 503.704010
        self.fy = 505.431824
        self.cx = 329.486994
        self.cy = 238.680561
        self.chair_height = 75 #centimeters, the previous values are pixels
        # ====================== CAMERA ELEMENTS ====================== #
        # ======================= ROSPY ELEMENTS ====================== #
        self.rate = rospy.Rate(10.0) 
        self.tf_buffer = tf2_ros.Buffer() #tf2 transform buffer
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)#tf2 listener to get the frame transforms
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10) #Marker (RViz) publisher
        self.detect_pub = rospy.Publisher("detect_orb_data", Float32MultiArray, queue_size=10) #Publisher for ORB-SLAM2 data, 5 float variables.
        rospy.Subscriber("/detectnet/detections", Detection2DArray, self.det_callback, queue_size=2)
        # ======================= ROSPY ELEMENTS ====================== #


    def det_callback(self, det_data):
        '''
        This function handles a bit too much. It handles the detected object filtering, the 
        depth calculation, and the marker position calculations and placement.
        Recently turned this into a class to more easily manage the different aspects of the code.
        '''
        #rospy.loginfo("Received data...")
        #rospy.loginfo(det_data.detections[0])
        # Choose only chairs.
        for elem in [x for x in det_data.detections if x.results[0].id == 62 and x.results[0].score >= 0.95]:
            # 10 for the cm to decimeters conversion, better for scale
            depth = (self.fy/elem.bbox.size_y)*self.chair_height/10 
            x = (elem.bbox.center.x  - self.cx) * (depth/self.fx)
            y = (elem.bbox.center.y - self.cy) * (depth/self.fy)
            rospy.loginfo(depth)
            rospy.loginfo("x: " + str(x))
            rospy.loginfo("y: " + str(y))
            rospy.loginfo("score: " + str(elem.results[0].score))

            # Send data to modified ORB-SLAM2
            float_data = Float32MultiArray()
            float_data.data = [depth, elem.bbox.center.x, elem.bbox.center.y, elem.bbox.size_y, elem.bbox.size_x]
            self.detect_pub.publish(float_data)

            # Fetch the transformation from the camera's frame to the map's frame.
            try:
                trans = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue

            # Create a marker for the chair 
            marker = Marker()
            # This should be probably changed to "/base_link" (not quite).
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            # Hacked transform. <3
            marker.pose.position.x = depth 
            marker.pose.position.y = -x 
            marker.pose.position.z = -y 
            marker.pose.orientation.w = 1.0
            # Other pretty variables
            marker.scale.x = 1.5
            marker.scale.y = 1.5  
            marker.scale.z = 2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.45
            marker.color.b = 0.8
            # Painful maths because there is a horrible lack of Python documentation for this specific matter.
            # Added rotation component from transform quaternion.
            pose_matrix = tf.transformations.quaternion_matrix([trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w])
            # Added the translation.
            pose_matrix[0,3] = trans.transform.translation.x
            pose_matrix[1,3] = trans.transform.translation.y
            pose_matrix[2,3] = trans.transform.translation.z
            # Did the woobly doobly maths with vector multiplication.
            marker_vector = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, 1])
            # This should work. Maybe. It still seems rather off.
            marker_vector_final = np.dot(pose_matrix, marker_vector)
            marker.pose.position.x = marker_vector_final[0]
            marker.pose.position.y = marker_vector_final[1] 
            marker.pose.position.z = marker_vector_final[2] 

            rospy.loginfo(marker_vector_final)
            #rospy.loginfo(pose_matrix)

            self.marker_pub.publish(marker)
            self.rate.sleep()
            

    def run(self):
        #marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        rospy.spin()


if __name__== '__main__':
    try:
        rospy.init_node("chair_calculator", anonymous=True)
        chair_magic = ChairMagic()
        chair_magic.daemon = True
        chair_magic.run()
    except rospy.ROSInterruptException:
        pass
