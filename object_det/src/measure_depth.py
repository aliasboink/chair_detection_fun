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
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge 
import numpy as np

''' 
Raspberry PI v2 640x480
projection
503.704010 0.000000 329.486994 0.000000
0.000000 505.431824 238.680651 0.000000
0.000000 0.000000 1.000000 0.000000
'''

'''
Known/hypothetical bugs:
1. Due to how I will check the bounds for proximity (absolute values) there can be
   errors in the sense that we cannot place another chair mirrored with 
   respect to the origin of another chair. We do not have the means to test this, but
   that is most likely the case based on how the proximity check was thought out
   (i.e., in a very simplistic fashion).
'''
'''
Performance enhancing tips:
1. Try to find the ROS function that does all the heavy lifting OR use quaternions
   instead of the rotation matrix for the change of frame of markers.
'''

class ChairDet():
    '''
    Effectively a 3D point for now, as that's all we need.
    '''
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z



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
        self.chair_height = 93 #this is in centimeters, the previous values are pixels
        # ====================== CAMERA ELEMENTS ====================== #
        # ======================= ROSPY ELEMENTS ====================== #
        self.marker_arr = MarkerArray()
        self.rate = rospy.Rate(15.0) 
        self.tf_buffer = tf2_ros.Buffer() #tf2 transform buffer
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer) #tf2 listener to get the frame transforms
        self.marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10) #Marker (RViz) publisher
        self.detect_pub = rospy.Publisher("detect_orb_data", Float32MultiArray, queue_size=10) #Publisher for ORB-SLAM2 data, 5 float variables.
        rospy.Subscriber("/detectnet/detections", Detection2DArray, self.det_callback, queue_size=2)
        # ======================= ROSPY ELEMENTS ====================== #
        # ======================= ALGORITHM ELEMENTS ================== #
        self.size = 20
        self.size_verify = 11
        self.verification_pass = float(self.size_verify)/self.size # also percentage
        self.margin_error = 0.2 # in decimeters
        self.proximity = 2.5 # should be in absolute values, decimeters!
        self.chair_detections = []
        self.valid_detections = []
        self.INITIALIZED = False
        self.count = 0
        # ======================= ALGORITHM ELEMENTS ================== #

    def check_marker_new(self, x, y):
        for marker in self.marker_arr.markers:
            m_x = abs(marker.pose.position.x)
            m_y = abs(marker.pose.position.y)
            if m_x-self.proximity < abs(x) < m_x+self.proximity or m_y-self.proximity < abs(y) < m_y+self.proximity:
                return False
        return True

    def check_error_bounds(self, chair, avg_x, avg_y, margin):
        '''
        Be ware that the ABSOLUTE VALUES of the ranges are given.
        avg_x and avg_y are the values we check against and we also take into account
        a certain margin of error or margin of distance
        '''
        '''
        rospy.loginfo("Chair verif avg X:" + str(avg_x))
        rospy.loginfo("Chair verif avg Y:" + str(avg_y))
        rospy.loginfo("Margin error verif:" + str(self.margin_error))
        '''
        x = abs(chair.x)
        y = abs(chair.y)
        if (avg_x-margin) < x < (avg_x+margin) and (avg_y-margin) < y < (avg_y+margin):
            return True
        return False
        
    
    def get_verification(self):
        '''
        Return a verification score and reinitialize the valid detections list for further use.
        chair - chair position in 3D
        '''
        sum_x = sum([elem.x for elem in self.chair_detections])
        sum_y = sum([elem.y for elem in self.chair_detections])
        avg_x = float(sum_x)/self.size # the length will always be equal to self.size
        avg_y = float(sum_y)/self.size
        self.valid_detections = [] # this will be used outside of the function, a bit "spaghetti"-esque
        #rospy.loginfo("In get_verification:")
        for chair in self.chair_detections:
            # We send in the absolute values for the bounds checking.
            if self.check_error_bounds(chair, abs(avg_x), abs(avg_y), self.margin_error):
                #rospy.loginfo("Appending to the valid detections...")
                self.valid_detections.append(chair)
        #rospy.loginfo(float(len(self.valid_detections)))
        return [float(len(self.valid_detections))/self.size, avg_x, avg_y]


    def det_callback(self, det_data):
        '''
        This function handles a bit too much. It handles the detected object filtering, the 
        depth calculation, and the marker position calculations and placement.
        Recently turned this into a class to more easily manage the different aspects of the code.
        The mathematics aren't as efficient as they could be computationally, but for the sake
        of "novice's clarity" I will leave it as is for now.
        '''
        #rospy.loginfo("Received data...")
        #rospy.loginfo(det_data.detections[0])

        # Choose only chairs.
        for elem in [x for x in det_data.detections if x.results[0].id == 62 and x.results[0].score >= 0.60]:

            # Reverse the projection using the now calculated depth.
            # division by 10 for the cm to decimeters conversion, better for scale
            depth = (self.fy/elem.bbox.size_y)*self.chair_height/10 
            x = (elem.bbox.center.x  - self.cx) * (depth/self.fx)
            y = (elem.bbox.center.y - self.cy) * (depth/self.fy)

            #rospy.loginfo("Depth: " + str(depth))
            # Send data to modified ORB-SLAM2
            float_data = Float32MultiArray()
            float_data.data = [depth, elem.bbox.center.x, elem.bbox.center.y, elem.bbox.size_y, elem.bbox.size_x]
            self.detect_pub.publish(float_data)

            # Fetch the transformation from the camera's frame to the map's frame.
            try:
                # The two frame ID's could be reversed. This seems to be from /base_link to /map
                # (which is what we want) based on the latest documentation and results.
                trans = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                rospy.loginfo("Transform not found, maybe ORB-SLAM ROS is not running?")
                continue

            # Painful maths because there is a horrible lack of Python documentation for this specific matter.
            # Added rotation component from transform quaternion.
            # (could've been done with quaternion mathematics, but the rotation matrix is more intuitive)
            pose_matrix = tf.transformations.quaternion_matrix([trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w])
            # Added the translation.
            pose_matrix[0,3] = trans.transform.translation.x
            pose_matrix[1,3] = trans.transform.translation.y
            pose_matrix[2,3] = trans.transform.translation.z
            # Did the woobly doobly maths with vector multiplication.
            # This is a homogenous 3D vector of the marker's position based on 
            # the inverse of the projection matrix and depth estimation.
            marker_vector = np.array([depth, -x, -y, 1]) # RVIZ (x,y,z,1) with a hacked transform from the camera values
            # This should work. Maybe. It still seems rather off.
            marker_vector_final = np.matmul(pose_matrix, marker_vector)

            if self.INITIALIZED is True: 
                
                # Printing for the sake of debugging
                '''
                rospy.loginfo(depth)
                rospy.loginfo("x: " + str(x))
                rospy.loginfo("y: " + str(y))
                rospy.loginfo("score: " + str(elem.results[0].score))
                '''

                # Dequeue and queue another detection's coordinates in chair_detections.
                self.chair_detections.pop(0)
                self.chair_detections.append(ChairDet(marker_vector_final[0], marker_vector_final[1], marker_vector_final[2]))
                # Get the verification score together with the valid_detections average
                # the new x, y are the averages after transform and 
                # the new z is just z after the transform
                verification_score, new_x, new_y = self.get_verification()
                new_z = float(sum([elem.z for elem in self.chair_detections]))/self.size
                #rospy.loginfo("Verification score: " + str(verification_score))
                #rospy.loginfo("Verification pass: " + str(self.verification_pass))

                
                if verification_score > self.verification_pass and self.check_marker_new(new_x, new_y):
                    # ============ MARKER BEAUTY START ===============#
                    # Create a marker for the chair 
                    rospy.loginfo("Creating a marker...................................")
                    marker = Marker()
                    # This should be probably changed to "/base_link" (not quite).
                    marker.header.frame_id = "/map"
                    marker.header.stamp = rospy.Time.now()
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    self.count = self.count + 1
                    marker.id = self.count
                    # Hacked transform from camera to RViz 3D space.
                    marker.pose.orientation.w = 1.0
                    # Marker aspect variables.
                    marker.scale.x = 1.5
                    marker.scale.y = 1.5  
                    marker.scale.z = 2
                    marker.color.a = 1.0
                    marker.color.r = 0
                    marker.color.g = 1
                    marker.color.b = 0
                    # ============= MARKER BEAUTY END ===============#
                    marker.pose.position.x = new_x
                    marker.pose.position.y = new_y
                    marker.pose.position.z = new_z 
                    # Debugging
                    rospy.loginfo(marker_vector_final)
                    rospy.loginfo([new_x, new_y])
                    rospy.loginfo(marker_vector)
                    
                    self.marker_arr.markers.append(marker)
                #rospy.loginfo("Length: " + str(len(self.marker_arr.markers)))
                self.marker_pub.publish(self.marker_arr)                
            else: 
                # Will initialize if the number of elements in the chair detections vector
                # reaches a size of "self.size"
                rospy.loginfo("Initializing...")
                self.chair_detections.append(ChairDet(marker_vector_final[0], marker_vector_final[1], marker_vector_final[2]))
                if len(self.chair_detections) == self.size: 
                    self.INITIALIZED = True

            self.rate.sleep()

            

    def run(self):
        #marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        rospy.spin()


if __name__== '__main__':
    try:
        rospy.init_node("chair_calculator", anonymous=True)
        chair_magic = ChairMagic()
        chair_magic.run()
    except rospy.ROSInterruptException:
        pass
