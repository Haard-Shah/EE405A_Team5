#!/usr/bin/python
import cv2
import rospy
import numpy as np


from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image

from pyzbar.pyzbar import decode

# detector = cv2.QRCodeDetector()

# Note: Let's just use TA's qr scanner
# roslaunch realsense2_camera rs_rgbd.launch
# rosrun nodel nodelet manager __name:=nodelet_manager
# rosrun nodelet nodelet load qr_detector/qr_detector_nodelet nodelet_manager
# It's way easier

bridge = CvBridge()

rospy.init_node('qr_node')
pub_riddle = rospy.Publisher('/riddle/question', String, queue_size = 10)
pub_qr = rospy.Publisher('/riddle/qr', Image, queue_size = 10)
rate = rospy.Rate(10.0)


def detect(image_msg):
    global bridge
    try:
        cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        decoded = decode(cv2_img)
        #print(decoded)
    except CvBridgeError, e:
        print(e)
    except:
        print("Something wrong!")
    else:
        if (decoded):
            for item in decoded:
                data = item.data
                print(data)
            print("Decode Finish!")

    """
    cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    retval, decoded_info, points, straight_qrcode = decoder.detectAndDecode(cv2_img)

    if(retval):
	print(decoded_info)
    else:
	print("FAILED TO DECODE!")
    """

if __name__ == '__main__':
    rospy.Subscriber('/qr', Image, detect)
    rospy.spin()
