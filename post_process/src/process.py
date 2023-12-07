#!/usr/bin/python
import cv2
import rospy
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from final_result_msgs.msg import save_image
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import *

bridge = CvBridge()

rospy.init_node('process_node')
pub_qr = rospy.Publisher('/qr', Image, queue_size = 10)
pub_submit = rospy.Publisher('/target_detection', save_image, queue_size = 10)
rate = rospy.Rate(10.0)

submitted = []

buffer = 100

# 3D rotation around y-axis.
def rotate_y(image, rad):
    height, width, _ = image.shape
    theta, phi, gamma = 0, rad, 0

    focal = np.sqrt(height ** 2 + width ** 2) / (2 * np.sin(gamma) if np.sin(gamma) != 0 else 1)
    dx, dy, dz = 5, 1, focal

    A1 = np.array([[1, 0, -1 * width / 2], 
                   [0, 1, -1 * height / 2],
                   [0, 0, 1],
                   [0, 0, 1]])
        
    RX = np.array([[1, 0, 0, 0],
                   [0, np.cos(theta), -1 * np.sin(theta), 0],
                   [0, np.sin(theta), np.cos(theta), 0],
                   [0, 0, 0, 1]])
        
    RY = np.array([[np.cos(phi), 0, np.sin(phi), 0],
                   [0, 1, 0, 0],
                   [np.sin(phi), 0, np.cos(phi), 0],
                   [0, 0, 0, 1]])
        
    RZ = np.array([[np.cos(gamma), -1 * np.sin(gamma), 0, 0],
                   [np.sin(gamma), np.cos(gamma), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
        
    R = np.dot(np.dot(RX, RY), RZ)

    T = np.array([[1, 0, 0, dx], 
                  [0, 1, 0, dy], 
                  [0, 0, 1, dz], 
                  [0, 0, 0, 1]])
        
    A2 = np.array([[focal, 0, width / 2, 0], 
                   [0, focal, height / 2, 0], 
                   [0, 0, 1, 0]])
        
    MAT = np.dot(A2, np.dot(T, np.dot(R, A1)))
    rotated = cv2.warpPerspective(image.copy(), MAT, (width, height))
    
    # cv2.imwrite("rotated.jpg", rotated)
    return rotated 

def filter(image, thres):
    image[np.all(image > (thres, 0, 0), axis=-1)] = (255,255,255)
    image[np.all(image > (0, thres, 0), axis=-1)] = (255,255,255)
    image[np.all(image > (0, 0, thres), axis=-1)] = (255,255,255)

    # cv2.imwrite(f"filtered.jpg", image)
    return image

def warp(image):
    width, height = 720, 1280

    pts1 = np.float32([[0, 0], [0, 0], [0, 0], [0, 0]])
    pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])

    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    warped = cv2.warpPerspective(image, matrix, (width, height))
    return warped

def listener(image_msg, bounding_msg):
    global bridge, submitted

    for box in bounding_msg.bounding_boxes:
        print("Processing Bound box:", box)
        if(box.Class != "APRIL_TAG" and box.Class != "QR_IMAGE"):
            if((int(box.Class) in submitted)):
                return
            
        #xcenter = int((box.xmin + box.xmax) / 2)
        #ycenter = int((box.ymin + box.ymax) / 2)
        #xdelta = int(1.5 * (box.xmax - box.xmin))
        #ydelta = int(1.5 * (box.ymax - box.ymin))
            
        if(box.probability > 0.8 and box.Class != "APRIL_TAG" and box.Class != "QR_IMAGE"):
            print("Detection Prob high, processing img")

            submission = save_image()
            submission.class_id = int(box.Class)

            try:
                cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
            except CvBridgeError as e:
                print(e)
            except:
                print("Something wrong!")
            else:

                # Draw the bounding box on the imag for debugging purposes
                # width = box.xmax - box.xmin
                # height = box.ymax - box.ymin

                # cv2_crop = cv2.rectangle(cv2_img, (box.xmin//2, box.ymin//2), (min(int(box.xmax*1.5), 720), min(int(box.ymax*1.5), 1280)), (255, 255, 0), 3)
                cv2_crop = cv2.rectangle(cv2_img, (max(box.xmin-buffer, 0), max(box.ymin-buffer, 0)), (min(box.xmax+buffer, 1280), min(box.ymax+buffer, 720)), (255, 255, 0), 3)
                # cv2_crop = cv2_img[box.ymin - buffer:box.ymax+buffer, box.xmin - buffer:box.xmax + buffer]
                # cv2_crop = cv2_img[:, box.xmin:box.xmax]
            try:
                print("Cropping image")
                submission.save_img = bridge.cv2_to_imgmsg(cv2_crop, "bgr8")
            except CvBridgeError as e:
                print(e)
            except:
                print("Something wrong!")
            else:
                submission.x_pose = 0
                submission.y_pose = 0
                pub_submit.publish(submission)
                submitted.append(int(box.Class))
                print("submitted ")
                print(submission.class_id)
                print(box.probability)

if __name__ == '__main__':
	# /darknet_ros/detection_image
    sync = ApproximateTimeSynchronizer([Subscriber('/camera/color/image_raw', Image), 
                                        Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)],
                                       queue_size = 5, slop = 0.1)
    print("Node running ...")
    sync.registerCallback(listener)
    rospy.spin()
