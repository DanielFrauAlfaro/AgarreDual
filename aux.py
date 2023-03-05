#! /usr/bin/python3

from __future__ import print_function
import dearpygui.dearpygui as dpg
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CompressedImage

from spatialmath import SE3
import roboticstoolbox as rtb

from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import time

pose = Pose()

or_x = 0.5137                        # Origen del movimiento cartesiano
or_y = 0.1334                       
or_z = 0.4397

or_roll = -2.296 # 0.832 # 0.87                # Origen del movimiento articular de la muñeca del robot
or_pitch = 0.0 # 1.12                   # YXZ hacia abajo # YXZ # XYZ
or_yaw = -3.03 # 1.12 # -0.001661

pose.position.x = or_x
pose.position.y = or_y
pose.position.z = or_z

pose.orientation.x = or_roll
pose.orientation.y = or_pitch
pose.orientation.z = or_yaw

def aux(data):
    global pose
    pose = data

rospy.init_node("aux")

pub = rospy.Publisher("/ur5_2/pose", Pose, queue_size=10)
rospy.Subscriber("/ur5_2/pose_aux", Pose, aux)

r = rospy.Rate(13)

while not rospy.is_shutdown():
    pub.publish(pose)
    r.sleep()


