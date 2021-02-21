#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from datetime import datetime
import time
import json
import requests

class judgedata():
    def __init__(self):
        self.path = "http://127.0.0.1:5000/warState"
        result = requests.get(self.path)
        self.bjudge = json.loads(result.text.encode())["targets"]
    def getdata(self):
        recdata = ""
        result = requests.get(self.path)
        if result.status_code == 200:
            text = result.text.encode()
            value = json.loads(text)    # 辞書型に変換
            # print text
            # print value
            # 得点保持者が変更になった時
            if value["targets"] != self.bjudge:
                # print judge["targets"]
                recdata = text
                self.bjudge = value["targets"]
 
        return recdata

if __name__ == '__main__':
    rospy.init_node('sendjudge')
    r = rospy.Rate(10) # 10hz 0.1s
    pub = rospy.Publisher('point_state', String)
    judge = judgedata()
    while not rospy.is_shutdown():
        data = judge.getdata()
        if not data == "":
            # print data
            pub.publish(data)
        r.sleep()
    