#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import collections
import os
from spatialmath import SE3
import roboticstoolbox as rtb
from math import pi
import sys
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import time
from ur5_class import UR5e

if __name__ == "__main__":
    rospy.init_node("master_pybullet")
    client = p.connect(p.GUI)

    n = int(sys.argv[1])
    names = [sys.argv[2], sys.argv[3]]
    origin = [[float(sys.argv[5]), float(sys.argv[7]), float(sys.argv[9])], 
              [float(sys.argv[11]), float(sys.argv[13]), float(sys.argv[15])]]
    grip = [sys.argv[16], sys.argv[17]]
    dir = sys.argv[18]

    ur = []

    for i in range(n):
        ur.append(UR5e(client=client, name=names[i], grip=grip[i], pos=origin[i], dir=dir))

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        for i in range(n):
            ur[i].get_observation()
            
        p.stepSimulation()
        r.sleep()
        

    