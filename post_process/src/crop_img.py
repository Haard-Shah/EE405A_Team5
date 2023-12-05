#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Darknet Message
from darknet_ros_msgs.msg import BoundingBoxes

# Instantiate CvBridge
bridge = CvBridge()

# Detected class
detected_nums = set()

# Current Image
image = None

def publish_image(cv2_file):
    image_pub = rospy.Publisher("/result_image",Image)
    try:
        # Convert your ROS Image message to OpenCV2
        ros_img = bridge.cv2_to_imgmsg(cv2_file, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Publish image
        image_pub.publish(ros_img)

def image_callback(msg):
    # print("Received an image!")
    global image
    image = msg
    """    
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image.jpeg', cv2_img)
    """

def box_callback(msg):
    # print("Received a target!")
    global image
    for i in range(len(msg.bounding_boxes)):
        target = msg.bounding_boxes[i].Class
	if (target not in detected_nums):
            detected_nums.add(target)   
	    xmin = msg.bounding_boxes[i].xmin
	    xmax = msg.bounding_boxes[i].xmax
	    ymin = msg.bounding_boxes[i].ymin
	    ymax = msg.bounding_boxes[i].ymax
            print("Class: {}, xmin: {}, xmax: {}, ymin:{}, ymax:{}".format(target, xmin, xmax, ymin, ymax))
            if(image):
                print("Image Available")
                image_name = "{}_image.jpeg".format(target)
                
                # Code below to save image
                try:
                    # Convert your ROS Image message to OpenCV2
                    cv2_img = bridge.imgmsg_to_cv2(image, "bgr8")
                except CvBridgeError, e:
                    print(e)
                else:
                    cv2_crop = cv2_img[ymin:ymax, xmin:xmax]
                    # Save your OpenCV2 image as a jpeg 
                    # cv2.imwrite(image_name, cv2_crop)
                    publish_image(cv2_crop)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/color/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    
    box_topic = "/darknet_ros/bounding_boxes"

    rospy.Subscriber(box_topic, BoundingBoxes, box_callback)

    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
