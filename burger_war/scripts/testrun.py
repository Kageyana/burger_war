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
        # フィールド得点の座標
        self.point = { 'Tomato_N':     [0.8,0.55,self.pi, 6],\
                        'Tomato_S':     [0.1,0.55,0, 7],\
                        'Omelette_N':   [0.8,-0.55,self.pi, 8],\
                        'Omelette_S':   [0.1,-0.55,0, 9],\
                        'Pudding_N':    [-0.1,0.55,self.pi,10],\
                        'Pudding_S':  [-0.8,0.55,0, 11],\
                        'OctopusWiener_N':[-0.1,-0.55,self.pi,12],\
                        'OctopusWiener_S':[-0.8,-0.55,0, 13],\
                        'FriedShrimp_N':[0.4,0,self.pi, 14],\
                        'FriedShrimp_E':[0,-0.4,self.pi/2, 15],\
                        'FriedShrimp_W':[0,0.4,-self.pi/2, 16],\
                        'FriedShrimp_S':[-0.4,0,0, 17],\
                        'wait':[-1,0,0, 0]\
                        }
        #　獲得する順番
        self.orderPoint = ['Pudding_S',\
                            'FriedShrimp_S',\
                            'Pudding_N',\
                            'FriedShrimp_W',\
                            'Tomato_S',\
                            'Tomato_N',\
                            'FriedShrimp_N',\
                            'Omelette_N',\
                            'Omelette_S',\
                            'FriedShrimp_E',\
                            'OctopusWiener_N',\
                            'OctopusWiener_S'\
                        ]
        # 現在のorderPointのインデックス
        self.order = 0
        # 現在の目標得点
        self.nowGoal = []

    def setGoal(self,xyyaw):
        self.client.wait_for_server()

        # 座標名keyから取得
        self.nowGoal = [k for k, v in self.point.items() if v == xyyaw][0]
        self.nowGoal = self.point[self.nowGoal]
        print self.nowGoal

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

    def subcallback_point(self,data):
        # 空リストではない時
        if self.nowGoal:
            # 辞書型に変換
            targets = json.loads(data.data)["targets"]
            # print target[0]
            # 現在の目標得点を獲得できたか確認
            if targets[self.nowGoal[3]]['player'] == "r":
                for i in range(12):
                    # 目標得点を更新
                    self.order = self.order + 1
                    if self.order >= 12:
                        self.order = 0
                    self.nowGoal = self.point[self.orderPoint[self.order]]
                    print self.orderPoint[self.order] , targets[self.nowGoal[3]]['player']
                    # print targets[int(self.nowGoal[3])], targets[int(self.nowGoal[3])]['player']
                    # 未獲得の得点かどうか確認
                    if targets[self.nowGoal[3]]['player'] != "r":
                        #　未獲得ならループ終了
                        break
                    elif targets[self.nowGoal[3]]['player'] == "r" and i == 11:
                        # すべて獲得していれば待機位置に戻る
                        self.nowGoal = self.point['wait']
                print self.orderPoint[self.order]
                self.setGoal(self.nowGoal)
    def subcallback_war(self,data):
        if self.nowGoal == self.point['wait']:
            self.order = 0
            targets = json.loads(data.data)["targets"]
            for i in range(12):
                if targets[i+6]['player'] != "b":
                    # 敵の得点があれば獲得しに行く
                    self.nowGoal = self.point[targets[i+6]['name']]
                    self.setGoal(self.nowGoal)
                    break
            

    def lidner(self):
        # war_stateを取得
        rospy.Subscriber('point_state', String, self.subcallback_point)
        rospy.Subscriber('war_state', String, self.subcallback_war)
        # トピック更新の待ちうけを行う
        # rospy.spin()

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        self.setGoal(self.point['Pudding_S'])
        while True:
            r.sleep()

    
if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.lidner()
    bot.strategy()