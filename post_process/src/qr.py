#!/usr/bin/python
import cv2
import rospy
import numpy as np


from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image

detector = cv2.QRCodeDetector()

rospy.init_node('qr_node')
pub_riddle = rospy.Publisher('/riddle/question', String, queue_size = 10)
pub_qr = rospy.Publisher('/riddle/qr', Image, queue_size = 10)
rate = rospy.Rate(10.0)


def detect(image_msg):
    cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    retval, decoded_info, points, straight_qrcode = decoder.detectAndDecode(cv2_img)

    if(retval):
	print(decoded_info)
    else:
	print("FAILED TO DECODE!")
    

if __name__ == '__main__':
    rospy.Subscriber('/qr', Image, detect)
    rospy.spin()
