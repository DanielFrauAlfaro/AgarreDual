#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float64, Float32MultiArray
import sys
from math import pi

# Se controla tanto la pinza de tres dedos como la de dos dedos

# Suscripciones:
#   - grip_pos: la posición de las articulaciones de la pinaz
#   - grip_torque: torque de las articulaciones de la pinza
#   - grip_cmd: recibir el comando de la pinza

# Publishers:
#   - command: topic para los comandos de cada dedo de cada pinza
#   - grip_state: publica el estado de la pinza [0, 255]

class GripperController():
    def __init__(self, name, model, finger):
        # ------ Publishers ------
        self.joint_pub = []

        if model == "3f":
            self.joint_pub.append(rospy.Publisher("/" + name + "/finger_" + finger + "_joint_1_controller/command", Float64, queue_size=10))
            self.joint_pub.append(rospy.Publisher("/" + name + "/finger_" + finger + "_joint_2_controller/command", Float64, queue_size=10))
            self.joint_pub.append(rospy.Publisher("/" + name + "/finger_" + finger + "_joint_3_controller/command", Float64, queue_size=10))

        elif model == "2f_140":
            self.joint_pub.append(rospy.Publisher("/" + name + "/gripper/command", Float64, queue_size=10))

        self.state = rospy.Publisher("/" + name + "/grip_state", Float32, queue_size=10)


        # ------ Subscribers ------
        rospy.Subscriber("/" + name + "/grip_cmd", Float32, self.cmd_cb)
        rospy.Subscriber("/" + name + "/grip_pos", Float32MultiArray, self.pos_cb)
        rospy.Subscriber("/" + name + "/grip_torque", Float32MultiArray, self.torque_cb)

        th = [110, 140, 240, 255]

    def cmd_cb(self, data):
        pass

    def pos_cb(self, data):
        pass

    def torque_cb(self, data):
        pass

if __name__ == "__main__":

    name = sys.argv[1]
    model = sys.argv[2]
    finger = int(sys.argv[3])

    grip_ctr = GripperController(name, model, finger)

    rospy.spin()