#!/usr/bin/env python3
 
'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of Krishi Bot (KB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
 
# Team ID:			[ 1884 ]
# Author List:		[CM Antony,Navneeth P Sagar,Tanishq Choudhary,Praneta Maheshwar]
# Filename:			KB_1884.py
# Functions:        [ laser_callback, control_loop]
# Nodes:		    [ebot_controller, cmd_vel, /ebot/laser/scan]
 
 
####################### IMPORT MODULES #######################
from asyncore import read
from multiprocessing.resource_sharer import stop
from re import T
import sys
import traceback
 
import random
 
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
 
##############################################################
 
################# ADD GLOBAL VARIABLES HERE #################
 
range_max = 8
regions = {
    'bright': range_max,
    'fright': range_max,
    'front': range_max,
    'fleft': range_max,
    'bleft': range_max,
}
tol = 5
f=0
 
 
##############################################################
 
################# ADD UTILITY FUNCTIONS HERE #################
 
def laser_callback(msg):
    global regions,f
    regions = {
        # 'rr': min(msg.ranges[0],range_max),
        'bright': min(min(msg.ranges[0:143]), range_max),
        'fright': min(min(msg.ranges[144:287]), range_max),
        'front': min(min(msg.ranges[288:431]), range_max),
        # 'ff': min(msg.ranges[355:364],range_max),
        'fleft': min(min(msg.ranges[432:575]), range_max),
        'bleft': min(min(msg.ranges[576:719]), range_max),
        # 'll': min(msg.ranges[0],range_max),
    }
    f=min((msg.ranges[359]), range_max)
 
 
def control_loop():
    wf = 0
    count = 0
    flag = 1
 
    rospy.init_node('ebot_controller', anonymous=True)
 
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)  # msg.ranges[0:720]
 
    rate = rospy.Rate(10)
 
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
 
    global regions
 
    prev_val = 0
    step_miss=0
    status = "Wall follow"
 
    rospy.Duration(5.0)
    
    while not rospy.is_shutdown():
 
        if regions['front'] > 0.6: 
 
            velocity_msg.linear.x = 0.6
 
            if regions['bright'] < 0.5 :
                velocity_msg.angular.z = 0.5
                wf = 1
            elif regions['bright']>0.55 and regions['bright']<1 :
                velocity_msg.angular.z = -0.5
                wf = 1
            else:
                velocity_msg.angular.z = 0.0
 
            if regions['bright'] > 1.2 and wf == 1   and regions['fright'] > 0.7   :
                count+=1
                print(str(count),"count")
                if count ==4:
                    rospy.sleep(1.05)
                    continue
                if count ==6:
                    velocity_msg.linear.x = 0
                    velocity_msg.angular.z = 0
                    pub.publish(velocity_msg)
                    rospy.signal_shutdown("Task Completed")

                rospy.sleep(0.7)
                velocity_msg.linear.x = 0
                velocity_msg.angular.z = 0
 
                wf = 2
 
            if wf==2 :
                velocity_msg.linear.x = 0.0
                velocity_msg.angular.z = -1
                pub.publish(velocity_msg)
                rospy.sleep(3.2)
                wf=3

            if regions['bright'] > 0.8 and regions['fright'] <1.1 and  regions['front'] < 1 :
                velocity_msg.angular.z = 0.5
 
            pub.publish(velocity_msg)
        else:
            print("Obstacle detected!!!")
            velocity_msg.linear.x = 0
            velocity_msg.angular.z = 0
            pub.publish(velocity_msg)
        prev_val = regions['bright']
        rate.sleep()
 
 ##############################################################
 
 
######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        control_loop()
 
    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()
 
    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")