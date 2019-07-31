#!/usr/bin/env python
# coding: UTF-8

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class DetectObject:

    is_detect_object = Bool()

    def __init__(self):

        # 初期化処理
        rospy.init_node('detect_object', anonymous=True)
        self.is_detect_object.data = False

        # LRF情報をSubscribe  障害物の有無をPublish
        rospy.Subscriber("scan_front", LaserScan, self.callback)
        self.pub = rospy.Publisher('detect_front_object',Bool, queue_size=1000)

    def callback(self,data):

        # 設定した範囲内にある点群の数をカウントし、点が4つ以上あれば障害物があると判定
        data_num = len(data.ranges)
        obj_count = 0

        #LRF極座標データを直行座標データに変換(x,y),if you can't avoid objects, please change these change parameters.
        for i in range(0,data_num-1):
            if data.ranges[i] < 4.0 and data.ranges[i] > 0.2 :
                x = data.ranges[i] * math.sin( data.angle_min + data.angle_increment * i )
                y = data.ranges[i] * math.cos( data.angle_min + data.angle_increment * i )
                if 0.0 < y and y < 2.0:
                    if -1.0 < x and x < 1.0:
                        obj_count += 1

        if obj_count > 3:
            self.is_detect_object.data = True
        else:
            self.is_detect_object.data = False

                
    def publish_loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.pub.publish(self.is_detect_object)
            rate.sleep()


if __name__ == '__main__':
    
    detecter = DetectObject()
    detecter.publish_loop()