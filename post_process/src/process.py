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
    width, height = 640, 480

    pts1 = np.float32([[0, 0], [0, 0], [0, 0], [0, 0]])
    pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])

    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    warped = cv2.warpPerspective(image, matrix, (width, height))
    return warped

def listener(image_msg, bounding_msg):
    global bridge

    for box in bounding_msg.bounding_boxes:

	xcenter = int((box.xmin + box.xmax) / 2)
	ycenter = int((box.ymin + box.ymax) / 2)
	xdelta = int(1.5 * (box.xmax - box.xmin))
	ydelta = int(1.5 * (box.ymax - box.ymin))
        
	if(box.probability > 0.9 and box.Class != "APRIL_TAG" and box.Class != "QR_IMAGE" and (xcenter > 75 and xcenter < 565)):
	    submission = save_image()
	    submission.class_id = int(box.Class)

	    try:
                cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
            except CvBridgeError, e:
                print(e)
            except:
                print("Something wrong!")
            else:

                cv2_crop = cv2_img[(ycenter -ydelta):(ycenter + ydelta), (xcenter - xdelta):(xcenter + xdelta)]
		try:
                    submission.save_img = bridge.cv2_to_imgmsg(cv2_crop, "bgr8")
                except CvBridgeError, e:
                    print(e)
                except:
                    print("Something wrong!")
                else:
	            submission.x_pose = 0
		    submission.y_pose = 0
	    	    pub_submit.publish(submission)
		    print("submitted ")
 	            print(submission.class_id)
		    print(box.probability)
	else:
	    if(box.probability > 0.5 and box.Class == "QR_IMAGE" and (xcenter > 75 and xcenter < 565)):
                try:
                    cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
                except CvBridgeError, e:
                    print(e)
                except:
                    print("Something wrong!")
                else:

                    cv2_crop = cv2_img[(ycenter -ydelta):(ycenter + ydelta), (xcenter - xdelta):(xcenter + xdelta)]
		    try:
                        save_img = bridge.cv2_to_imgmsg(cv2_crop, "bgr8")
                    except CvBridgeError, e:
                        print(e)
                    except:
                        print("Something wrong!")
                    else:
	                pub_qr.publish(save_img)
 	                print("Get QR!")

if __name__ == '__main__':
    sync = ApproximateTimeSynchronizer([Subscriber('/camera/color/image_raw', Image), 
                                        Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)],
                                       queue_size = 5, slop = 0.1)
    sync.registerCallback(listener)
    rospy.spin()
