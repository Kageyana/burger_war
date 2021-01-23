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
        result = requests.get("http://127.0.0.1:5000/warState")
        self.bjudge = json.loads(result.text)["targets"]
    def getdata(self):
        recdata = {}
        result = requests.get("http://127.0.0.1:5000/warState")
        if result.status_code == 200:
            value = json.loads(result.text)
            # 得点保持者が変更になった時
            if value["targets"] != self.bjudge:
                # print judge["targets"]
                recdata = value["targets"]
                self.bjudge = value["targets"]
 
        return recdata

if __name__ == '__main__':
    rospy.init_node('sendjudge')
    r = rospy.Rate(10) # 10hz
    pub = rospy.Publisher('point_state', String)
    judge = judgedata()
    while not rospy.is_shutdown():
        data = judge.getdata()
        if not data == {}:
            # print data
            pub.publish(str(data))
        r.sleep()
    