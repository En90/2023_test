#!/usr/bin/env python3

import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from sample_function import *
import numpy as np

def talker(event):
    pub = rospy.Publisher("Sample_position", Float32MultiArray, queue_size = 1)
    # [6*5]: sample_n * [x, y, ang_z, ang_w, exist] 
    data = Float32MultiArray()

    for i in range(sample.sample_n):
        sum_x = 0
        sum_y = 0
        sum_ang_z = 0
        sum_ang_w = 0
        n = 0
        for j in range(sample.camera_n):
            if(sample.observed[j][i].exist==True):
                sum_x += sample.observed[j][i].x
                sum_y += sample.observed[j][i].y
                sum_ang_z += sample.observed[j][i].ang_z
                sum_ang_w += sample.observed[j][i].ang_w
                n+=1
        if n != 0:
            data.data.append(sum_x/n)
            data.data.append(sum_y/n)
            data.data.append(sum_ang_z/n)
            data.data.append(sum_ang_w/n)
            data.data.append(int(bool(n)))
        else:
            for k in range(5):
                data.data.append(0)

    pub.publish(data)
    # sample.listExist()
    sample.clear()

# callback function
def callback1(data):
    sample.find(0, data.pose.position.x, data.pose.position.y, data.pose.orientation.z, data.pose.orientation.w, data.pose.position.z,)
    # print("camera 0: ",end = '')
    # print(data.pose.position.x, data.pose.position.y, data.pose.orientation.z, data.pose.orientation.w)

def callback2(data):
    sample.find(1, data.pose.position.x, data.pose.position.y, data.pose.orientation.z, data.pose.orientation.w, data.pose.position.z,)
    # print("camera 1: ",end = '')
    # print(data.pose.position.x, data.pose.position.y, data.pose.orientation.z, data.pose.orientation.w)

### ros initialize
rospy.init_node("SamplePos")
rate = rospy.Rate(2)
sub1 = rospy.Subscriber("/aruco_single_central/pose",PoseStamped,callback1)
# sub2 = rospy.Subscriber("/aruco_single/pose2",PoseStamped,callback2)

sample = Sample()


rospy.Timer(rospy.Duration(2), talker, oneshot = False)

while not rospy.is_shutdown():

    # sample.listExist()
    sample.print()

    # print(sample[0])

    rate.sleep()
