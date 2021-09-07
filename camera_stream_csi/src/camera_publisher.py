#!/usr/bin/python2
import cv2
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# A bit ambitious with the capture resolution, but this is for the sole purpose of
# making use of the full FOV.
def gstreamer_pipeline(
    capture_width=1640,
    capture_height=1232,
    display_width=640,
    display_height=480,
    framerate=10,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def main(args = None):
    pub = rospy.Publisher('/cam1/image_raw', Image, queue_size=10)
    rospy.init_node("video_streamer", anonymous=True)
    rate = rospy.Rate(10)
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER) #CSI
    #ap = cv2.VideoCapture("/dev/video1") #USB
    br = CvBridge()
    
    while not rospy.is_shutdown():
        ret_val, img = cap.read()
        if ret_val == True:
            #cv2.imshow("Test",img)
            #cv2.waitKey(1)
            img_ros = br.cv2_to_imgmsg(img, "bgr8")
            img_ros.header.frame_id = "my_camera"
            img_ros.header.stamp = rospy.Time.now()
            pub.publish(img_ros)
            rospy.loginfo("Frame sent...")


        #rate.sleep()




if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
