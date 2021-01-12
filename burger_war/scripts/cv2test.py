#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import tf
# navigation
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
# opencv
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np




class NaviBot():
    def __init__(self):
        # velocity publisher
        #self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        #self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # camera subscribver
        # for convert image topic to opencv obj
        self.img = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        self.colimg = self.img
        # RGB抽出
        self.bgrLower = np.array([0, 0, 98])    # 抽出する色の下限
        self.bgrUpper = np.array([6, 6, 255])    # 抽出する色の上限
        self.img_mask = cv2.inRange(self.colimg, self.bgrLower, self.bgrUpper)
        self.colimg = cv2.bitwise_and(self.colimg, self.colimg, mask=self.img_mask)
        """
        self.gryimg = cv2.cvtColor( self.img, cv2.COLOR_BGR2GRAY )
        self.colimg = self.gryimg
        circles = cv2 . HoughCircles( self.gryimg, cv2.HOUGH_GRADIENT, 1, 20, param1=500, param2=30, minRadius=0, maxRadius=0 )
        if circles != None:
            circles = np.uint16 (np.around(circles))
            for i in circles [0,:] :
            # draw the outer circle
                cv2.circle( self.colimg, ( i[0] , i[1] ), i[2] , (0, 255, 0 ), 2)
                # draw the center of the circl
                cv2.circle( self.colimg, ( i[0] , i[1] ), 2, (0, 0, 255 ), 3)
                """
        cv2.imshow("Image window", self.colimg)
        cv2.waitKey(1)
        #cv2.destroyAllWindows()

    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        '''
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.a = 1
            

    
if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()