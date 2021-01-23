#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
import tf
import json
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

    def subcallback(self,data):
        judge = json.loads(data.data)
        for value in judge["targets"]:
            # フィールド得点を抽出
            if "BL" not in value["name"]:
                if  "RE" not in value["name"]:
                    print value["name"]+"   "+str(value["player"])

    def adder(self):
        # war_stateを取得
        rospy.Subscriber('war_state', String, self.subcallback)
        # トピック更新の待ちうけを行う
        # rospy.spin()

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
    # bot.adder()
    bot.strategy()