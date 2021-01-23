#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from datetime import datetime
import time
import json

class judgedata():
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
        rospy.spin()

if __name__ == '__main__':
    # node開始
    rospy.init_node('getjudge')
    # 1秒ごとに送信
    r = rospy.Rate(1)
    judge = judgedata()
    judge.adder()
    while not rospy.is_shutdown():
        r.sleep()
    