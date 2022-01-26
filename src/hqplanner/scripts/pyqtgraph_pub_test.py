#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np


pub = rospy.Publisher('data1d', Float32MultiArray, queue_size=10)
rospy.init_node('sender')
r = rospy.Rate(10)
data1d = Float32MultiArray()
t = np.linspace(0, 2.0*np.pi, 100)
ofs = 0.0
cnt = 0.0
while not rospy.is_shutdown():
    cnt = cnt+1.0
    cnt = np.mod(cnt, 100.0)
    ofs = 2.0*np.pi/100.0*cnt
    data1d.data = list(np.sin(t+ofs))
    pub.publish(data1d)
    r.sleep()