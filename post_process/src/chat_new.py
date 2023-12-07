#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# requests for chat
import requests
# ROS String message
from std_msgs.msg import String
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
# re for searching first instance of number
import re

# Final Result Message
from final_result_msgs.msg import save_image

# Current Image
image = None

rospy.init_node('chat_node')
pub_submit = rospy.Publisher('/target_detection', save_image, queue_size=10)
rate = rospy.Rate(10.0)

api = "https://api.openai.com/v1/chat/completions"
headers = {"Content-Type": "application/json", "Authorization": "Bearer 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"}

def image_callback(msg):
    # print("Received an image!")
    global image
    image = msg.data

def qr_callback(msg):
    # print("Received a target!")
    global image
    riddle = msg.data
    print(riddle)

    if len(riddle) == 0:
        return

    answer = None
    while(answer == None):
	payload = {"model": "gpt-3.5-turbo", "messages": [{"role": "user", "content": "Do not say anything but decimal integers above nine.\n\n" + riddle}], "temperature": 0}
	completion = requests.post(api, headers = headers, json = payload)
        response = completion.json()["choices"][0]["message"]["content"]
	answer = re.search(r'\d+', response).group()
        #answer = response
        
        if answer != None:
            print("RIDDLE_SOLVED: ")
	    print(answer)
	    submission = save_image()
	    submission.class_id = answer
	    submission.save_img = image
            pub_submit.publish(submission)
            break
	else:
	    print("FAILED!")
	    break

def main():
    # Define your image topic
    image_topic = "/qr"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    
    qr_topic = "/qr_codes"

    rospy.Subscriber(qr_topic, String, qr_callback)

    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
