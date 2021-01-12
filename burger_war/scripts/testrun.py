#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
import tf
# opencv
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# navigation
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs


class NaviBot():
    def __init__(self):
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # camera subscribver
        # for convert image topic to opencv obj
        #self.img = None
        #self.gryimg = None
        #self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result

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

    def subcallback(self,data):
        # 受けとったmessageの中身を足し算して出力
        print data.data

    def adder(self):
        #rospy.init_node('adder', anonymous=True)

        # Subscriberとしてimage_dataというトピックに対してSubscribeし、トピックが更新されたときは
        # callbackという名前のコールバック関数を実行する
        rospy.Subscriber('war_state', String, self.subcallback)

        # トピック更新の待ちうけを行う関数
        rospy.spin()

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        pi = 3.1415

        while True:
            self.setGoal(-0.8,0.4,0)

            self.setGoal(-0.4,0,0)
            
            self.setGoal(0,0.5,pi)
            self.setGoal(0,0.5,-pi/2)
            self.setGoal(0,0.5,0)

            self.setGoal(0.8,0.4,pi)

            self.setGoal(0.4,0,pi)

            self.setGoal(0.8,-0.4,pi)
            
            self.setGoal(0,-0.5,0)
            self.setGoal(0,-0.5,pi/2)
            self.setGoal(0,-0.5,pi)

            self.setGoal(-0.8,-0.4,0)

            r.sleep()

    
if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.adder()
    bot.strategy()