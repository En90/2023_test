#!/usr/bin/env python3
#coding: utf-8
from geometry_msgs.msg import PoseStamped
from random import random
import rospy
from time import time
rospy.init_node("sample_pub")
rate = rospy.Rate(3)
t_start = time()

def publish(pub, x, y, ang, color):
    data = PoseStamped()
    data.pose.position.x = x + (random()-0.5)
    data.pose.position.y = y + (random()-0.5)
    data.pose.orientation.z = ang + (random()-0.5)
    data.pose.orientation.w = color 
    pub.publish(data)



n = 0

while not rospy.is_shutdown():
    x = 1200 + random() * 350
    if n%2 == 0:
        y = 800 + random() * 350
    else:
        y = 1850 + random() * 350
    n+=1

    ang = random()*360
    color = int(random() * 3)
    
    pub1 = rospy.Publisher('/aruco_single/pose',PoseStamped, queue_size = 10)
    # pub2 = rospy.Publisher('/aruco_single/pose2',PoseStamped, queue_size = 10)

    publish(pub1, 1300 , 2000, ang, color)
    # publish(pub2, x ,y, ang, color)


    rate.sleep()
    
    print("is publishing... ",end = '')
    print("t=%.2f " % (time()-t_start))
