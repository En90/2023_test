#!/usr/bin/env python3

import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from sample_function import *
import numpy as np


def talker(event):
    pub = rospy.Publisher("Sample_position", Float32MultiArray, queue_size = 1)   #Sample_position
    # [6*5]: sample_n * [x, y, ang_z, ang_w, exist] 
    data = Float32MultiArray()

    sample.filter()
    sample.print()
    sample.clear()
    for i in range(sample.sample_n):
        for j in range(4):
            data.data.append(sample.data_avg[i][j])
        data.data.append(int(bool(sample.data_avg[i][0])))

    pub.publish(data)


# callback function
def callback1(data):
    if data.pose.position.z!=3:
    	sample.find(data.pose.position.x, data.pose.position.y, data.pose.orientation.z, data.pose.orientation.w, data.pose.position.z)
    # print("camera 0: ",end = '')
    # print(data.pose.position.x, data.pose.position.y, data.pose.orientation.z, data.pose.orientation.w)

def callback2(data):
    
    sample.find(data.pose.position.x, data.pose.position.y, data.pose.orientation.z, data.pose.orientation.w, data.pose.position.z)
    # print("camera 1: ",end = '')
    # print(data.pose.position.x, data.pose.position.y, data.pose.orientation.z, data.pose.orientation.w)

### ros initialize
rospy.init_node("SamplePos")
rate = rospy.Rate(1)
sub1 = rospy.Subscriber("/aruco_single_central/pose",PoseStamped,callback1)
# sub2 = rospy.Subscriber("/aruco_single/pose2",PoseStamped,callback2)

sample = Sample()


rospy.Timer(rospy.Duration(2), talker, oneshot = False)

while not rospy.is_shutdown():

    # sample.listExist()
    # sample.print()
    # print(sample.observed[0][5].x, sample.observed[0][5].y)
    # for i in range(6):
    #     print(sample.n[i])

    # print(sample[0])

    rate.sleep()
