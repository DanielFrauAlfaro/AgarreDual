#! /usr/bin/python3

import cv2
import numpy as np
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose

size = (550, 850, 3)

name_1 = "ur5_1"
name_2 = "ur5_2"

state1 = 0
state2 = 0

prev_x1, prev_y1, prev_z1 = 0.5095, 0.1334, 0.7347
prev_roll1, prev_pitch1, prev_yaw1 = 1.57225, 1.07, -0.00166

prev_x2, prev_y2, prev_z2 = 0.5095, 0.1334, 0.7347
prev_roll2, prev_pitch2, prev_yaw2 = 1.57225, 1.07, -0.00166

modes = []
modes.append("Position")
modes.append("Orientation")
modes.append("Gripper")

zeros = np.zeros(size, dtype="uint8")

def cart_pos1(data):
    global prev_x1, prev_y1, prev_z1
    global prev_roll1, prev_pitch1, prev_yaw1

    prev_x1 = data.position.x
    prev_y1 = data.position.y
    prev_z1 = data.position.z

    prev_roll1 = data.orientation.x
    prev_pitch1 = data.orientation.y
    prev_yaw1 = data.orientation.z

def cart_pos2(data):
    global prev_x2, prev_y2, prev_z2
    global prev_roll2, prev_pitch2, prev_yaw2

    prev_x2 = data.position.x
    prev_y2 = data.position.y
    prev_z2 = data.position.z

    prev_roll2 = data.orientation.x
    prev_pitch2 = data.orientation.y
    prev_yaw2 = data.orientation.z

def state1_cb(data):
    global state1

    state1 = data.data

def state2_cb(data):
    global state2

    state2 = data.data

rospy.init_node("interface")
rospy.Subscriber("/" + name_1 + "/state", Int32, state1_cb)
rospy.Subscriber("/" + name_2 + "/state", Int32, state2_cb)

rospy.Subscriber("/" + name_1 + "/cart_pos", Pose, cart_pos1)
rospy.Subscriber("/" + name_2 + "/cart_pos", Pose, cart_pos2)


while not rospy.is_shutdown():
    cv2.imshow("Interface", zeros)

    zeros = cv2.putText(zeros, "Mode UR5_1: " + modes[state1], (15,50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 0, 128), 2, cv2.LINE_AA)
    zeros = cv2.putText(zeros, "Mode UR5_2: " + modes[state2], (500,50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 0, 128), 2, cv2.LINE_AA)

    zeros = cv2.putText(zeros, "X: " + str(round(prev_x1,2)) + "   Y: " + str(round(prev_y1,2)) + "   Z: " + str(round(prev_z1,2)), (15,100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 0), 1, cv2.LINE_AA)
    zeros = cv2.putText(zeros, "Roll: " + str(round(prev_roll1,2)) + "  Pitch: " + str(round(prev_pitch1,2)) + "  Yaw: " + str(round(prev_yaw1,2)), (15,150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 0), 1, cv2.LINE_AA)
    
    zeros = cv2.putText(zeros, "X: " + str(round(prev_x2,2)) + "   Y: " + str(round(prev_y2,2)) + "   Z: " + str(round(prev_z2,2)), (500,100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 0), 1, cv2.LINE_AA)
    zeros = cv2.putText(zeros, "Roll: " + str(round(prev_roll2,2)) + "  Pitch: " + str(round(prev_pitch2,2)) + "  Yaw: " + str(round(prev_yaw2,2)), (500,150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 0), 1, cv2.LINE_AA)
    

    cv2.waitKey(1)