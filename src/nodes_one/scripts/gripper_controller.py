#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from gazebo_msgs.msg import ContactsState
from control_msgs.msg import JointControllerState
from math import pi


# Controller class
class GripperController():
    def __init__(self, name, model):
        
        # Model: 3f or 2f_140
        self.model = model


        # Publishers and subscribers for each gripper model
        self.joint_pub = []
        self.cmd = []

        if self.model == "3f":
            self.joint_pub.append(rospy.Publisher("/" + name + "/gripper_1_controller/command", Float64MultiArray, queue_size=5))
            self.joint_pub.append(rospy.Publisher("/" + name + "/gripper_2_controller/command", Float64MultiArray, queue_size=5))
            self.joint_pub.append(rospy.Publisher("/" + name + "/gripper_middle_controller/command", Float64MultiArray, queue_size=5))
            
            self.cmd = [0.0, 0.0, 0.0]


        elif model == "2f_140":
            self.joint_pub.append(rospy.Publisher("/" + name + "/gripper/command", Float64, queue_size=10))
            self.cmd = [0.0]

        # Subscriber for the command topic
        rospy.Subscriber("/" + name + "/grip_cmd", Float64, self.cmd_cb)
        

        # State machine thresholds
        self.th = [110, 140, 240, 255]


        # Maximum and minimum rotation (in radians)
        self.max_rot = [1.22, pi/2.0, -0.0523]
        self.min_rot = [0.0495, 0.0, -0.96]

        # Constants
        self.m1 = self.max_rot[0] / self.th[1] 
        self.m2 = self.max_rot[1] / (self.th[2] - self.th[1])

        
    # Command callback
    def cmd_cb(self, data):
        
        # Gets the data
        g = data.data

        # According to the model applies an action
        if self.model == "3f":
            
            # Control without obstacles
            if g >= 0.0 and g <= self.th[1]:
                self.cmd[0] = self.m1 * g
                self.cmd[1] = self.min_rot[1]
                self.cmd[2] = max(-self.m1 * g, self.min_rot[2])

            elif g <= self.th[2]:
                self.cmd[0] = self.max_rot[0]
                self.cmd[1] = self.m2 * (g - self.th[1])
                self.cmd[2] = self.min_rot[2]

            else:
                self.cmd[0] = self.max_rot[0]
                self.cmd[1] = self.max_rot[1]
                self.cmd[2] = self.min_rot[2]

            msg = Float64MultiArray()
            msg.data = self.cmd

            self.joint_pub[0].publish(msg)
            self.joint_pub[1].publish(msg)
            self.joint_pub[2].publish(msg)

        elif self.model == "2f_140":
            self.cmd[0] = data.data

            self.joint_pub[0].publish(self.cmd[0])