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
        self.pi = 3.1415
        self.point = {'Pudding_S':[-0.8,0.4,0],\
                    'FriedShrimp_S':[-0.4,0,0],\
                    'Pudding_N':[0,0.5,self.pi],\
                    'FriedShrimp_W':[0,0.5,-self.pi/2],\
                    'Tomato_S':[0,0.5,0],\
                    'Tomato_N':[0.8,0.4,self.pi],\
                    'FriedShrimp_N':[0.4,0,self.pi],\
                    'Omelette_N':[0.8,-0.4,self.pi],\
                    'Omelette_S':[0,-0.5,0],\
                    'FriedShrimp_E':[0,-0.5,self.pi/2],\
                    'OctopusWiener_N':[0,-0.5,self.pi],\
                    'OctopusWiener_S':[-0.8,-0.4,0],\
                    }

    def setGoal(self,xyyaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = xyyaw[0]
        goal.target_pose.pose.position.y = xyyaw[1]

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,xyyaw[2])        
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
        print data.data
        # judge = json.loads(data.data)
        # if judge["targets"] != self.bjudge:
        #     print judge["targets"]

    def adder(self):
        # war_stateを取得
        rospy.Subscriber('point_state', String, self.subcallback)
        # トピック更新の待ちうけを行う
        # rospy.spin()

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps

        while True:
            self.setGoal(self.point['Pudding_S'])

            self.setGoal(self.point['FriedShrimp_S'])
            
            self.setGoal(self.point['Pudding_N'])
            self.setGoal(self.point['FriedShrimp_W'])
            self.setGoal(self.point['Tomato_S'])

            self.setGoal(self.point['Tomato_N'])

            self.setGoal(self.point['FriedShrimp_N'])

            self.setGoal(self.point['Omelette_N'])
            
            self.setGoal(self.point['Omelette_S'])
            self.setGoal(self.point['FriedShrimp_E'])
            self.setGoal(self.point['OctopusWiener_N'])

            self.setGoal(self.point['OctopusWiener_S'])

            r.sleep()

    
if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.adder()
    bot.strategy()