#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 2.2 of Krishi Bot (KB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:			[ KB-1884]
# Author List:		[ CM Antony,Navneeth P Sagar,Tanishq,Praneta]
# Filename:			percepStack.py
# Functions:
# 					[ img_clbck,depth_clbck,image_processing,main ]


####################### IMPORT MODULES #######################
import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import time

# You can add more if required
##############################################################
global cx,cy,pub_rgb,pub_depth, pose, sub_image_color, sub_image_depth,flag
pose =[]
cx=0
cy=0
pub_rgb = rospy.Publisher('/center_rgb', String, queue_size=1)
pub_depth = rospy.Publisher('/center_depth', String, queue_size=1)
# Initialize Global variables
bridge = CvBridge()
flag=0
################# ADD UTILITY FUNCTIONS HERE #################

##############################################################

def img_clbck(img_msg):
    '''
    Callback Function for RGB image topic

    Purpose:
    -----
    Convert the image in a cv2 format and then pass it
    to image_processing function by saving to the
    'image' variable.

    Input Args:
    -----
    img_msg: Callback message.
    '''
    global pub_rgb,flag# , add global variable if any

    ############################### Add your code here #######################################
    #
    # pim = im.open(img_msg)
    # nimg = np.array(pim)
    # ocvim = cv2.cvtColor(nimg, cv2.COLOR_RGB2BGR)
    # img = "new.jpg"
    # cv2.imwrite(img, ocvim)

    # pub_rgb = rospy.Publisher('/center_rgb', String, queue_size=1)
    # pub_depth = rospy.Publisher('/center_depth', String, queue_size=1)

    img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')


    ##########################################################################################
    pose = image_processing(img)
    pub_rgb.publish(str(pose))
    flag = 1
    sub_image_color.unregister()

def depth_clbck(depth_msg):

    '''
    Callback Function for Depth image topic

    Purpose:
	---
    1. Find the depth value of the centroid pixel returned by the
    image_processing() function.
    2. Publish the depth value to the topic '/center_depth'


    NOTE: the shape of depth and rgb image is different.

    Input Args:
    -----
    depth_msg: Callback message.
    '''
    global cx, cy,pub_depth, pose,flag
    depth_val = []
    ############################### Add your code here #######################################
    imgd=bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    depth_array = np.array(imgd, dtype=np.float32)
    print(str(np.shape(depth_array)),"depth array")
    if len(pose)!=0:
        for cntr in pose:
            print(str(cntr[0]), str(cntr[1]), "cx,cy")
            d_v = depth_array[int(cntr[0] * 2 / 3)][int(cntr[1] * 0.6625)]
            depth_val.append(d_v)
            print(str(depth_val), "depth val")

    ##########################################################################################

    pub_depth.publish(str(depth_val))
    flag = 0
    sub_image_depth.unregister()

def image_processing(img):
    '''
    NOTE: Do not modify the function name and return value.
          Only do the changes in the specified portion for this
          function.
          Use cv2.imshow() for debugging but make sure to REMOVE it before submitting.

    1. Find the centroid of the bell pepper(s).
    2. Add the x and y values of the centroid to a list.
    3. Then append this list to the pose variable.
    3. If multiple fruits are found then append to pose variable multiple times.

    Input Args:
    ------
    image: Converted image in cv2 format.

    Example:
    ----
    pose = [[x1, y1] , [x2, y2] ...... ]
    '''
    global cx,cy,pub_rgb,pub_depth, pose
    pose = []
    contours1=0
    contours2=0

    ############### Write Your code to find centroid of the bell peppers #####################
    image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    low_colour1 = np.array([149, 102, 72])
    high_colour1 = np.array([179, 255, 255])
    low_colour2 = np.array([6, 124,111])
    high_colour2 = np.array([180, 255, 255])
    colour_filter1 = cv2.inRange(image, low_colour1, high_colour1)
    contours1, _ = cv2.findContours(colour_filter1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # contours1 = sorted(contours1, key=lambda x: cv2.contourArea(x), reverse=True)
    colour_filter2 = cv2.inRange(image, low_colour2, high_colour2)
    contours2, _ = cv2.findContours(colour_filter2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # contours2 = sorted(contours2, key=lambda x: cv2.contourArea(x), reverse=True)
    # cv2.drawContours(img,contours1,-1,(0,255,0),3)
    # cv2.drawContours(img, contours2, -1, (0, 255, 0), 3)
    # cv2.imshow("Image", img)
    # print(contours1)
    for j in contours1:
        if cv2.contourArea(j) > 2000:
            k = cv2.moments(j)
            if k['m00'] != 0:
                cx=int(k['m10'] / k['m00'])
                cy=int(k['m01'] / k['m00'])
                cv2.drawContours(img, [j], -1, (0, 255, 0), 2)
                cv2.circle(img, (cx, cy), 7, (0, 0, 0), -1)
                pose.append([cx,cy])

    for j in contours2:
        if cv2.contourArea(j) > 2000:
            k= cv2.moments(j)
            if k['m00'] != 0:
                cx =int(k['m10'] / k['m00'])
                cy= int(k['m01'] / k['m00'])
                cv2.drawContours(img, [j], -1, (0, 255, 0), 2)
                cv2.circle(img, (cx, cy), 7, (0, 0, 0), -1)
                pose.append([cx,cy])
    array=np.asarray(img, dtype="int32")
    print(str(np.shape(array)),"RGB image")
    print(pose)


    ##########################################################################################
    return pose
def main():
    '''
    MAIN FUNCTION

    Purpose:
    -----
    Initialize ROS node and do the publish and subscription of data.

    NOTE: We have done the subscription only for one image, you have to iterate over
    three images in the same script and publish the centroid and depth in the
    same script for three images, calling the same callback function.

    '''
    global sub_image_color, sub_image_depth,flag
    #### EDIT YOUR CODE HERE FOR SUBSCRIBING TO OTHER TOPICS AND TO APPLY YOUR ALGORITHM TO PUBLISH #####

    a="/device_0/sensor_1/Color_0/image/data_"
    b="/device_0/sensor_0/Depth_0/image/data_"
    rospy.init_node("percepStack", anonymous=True)
    # sub_image_color = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_2", Image, img_clbck)
    # sub_image_depth = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_2", Image, depth_clbck)


    # if flag == 0:
    #     sub_image_color = rospy.Subscriber(a + "2", Image, img_clbck)
    #     time.sleep(5)
    #
    # if flag == 1:
    #     sub_image_depth = rospy.Subscriber(b + "2", Image, depth_clbck)
    #     time.sleep(5)
    for j in range(1, 4):

        print(str(j))
        print(a + str(j))
        if flag==0:
            sub_image_color = rospy.Subscriber(a + str(j), Image, img_clbck)
            time.sleep(1.5)
        if flag == 1:
            sub_image_depth = rospy.Subscriber(b + str(j), Image, depth_clbck)
            time.sleep(1.5)
    rospy.signal_shutdown("fin")
    ####################################################################################################
    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")