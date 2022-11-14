#! /usr/bin/python3

from spatialmath import *
import roboticstoolbox as rtb
from math import pi
import matplotlib as plt
import cv2 as cv
import numpy as np
import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from pynput import keyboard as kb

# Main class for the controller
class Controller():
    def __init__(self):
        # UR5 model in Robotic Toolbox
        self.__ur5 = rtb.models.DH.UR5()
        self.__ur5.base = SE3.RPY(0,0,pi)      # Rotate robot base so it matches Gazebo model
        
        # Joint position vector: actual and home
        self.__q = [0, -1.5, 1 , 0.0, 0.0, 0.0]
        self.__q0 = self.__q
        
        # ROS parameters: node, publishers and subscribers
        rospy.init_node("main_controller", anonymous=False)
        
        self.__rate = rospy.Rate(10)        # Rate
        
        self.__joints_com = []
        self.__joints_com.append(rospy.Publisher('/shoulder_pan_joint_position_controller/command', Float64, queue_size=10))
        self.__joints_com.append(rospy.Publisher('/shoulder_lift_joint_position_controller/command', Float64, queue_size=10))
        self.__joints_com.append(rospy.Publisher('/elbow_joint_position_controller/command', Float64, queue_size=10))
        self.__joints_com.append(rospy.Publisher('/wrist_1_joint_position_controller/command', Float64, queue_size=10))
        self.__joints_com.append(rospy.Publisher('/wrist_2_joint_position_controller/command', Float64, queue_size=10))
        self.__joints_com.append(rospy.Publisher('/wrist_3_joint_position_controller/command', Float64, queue_size=10))

        # self.joints_state = []
        # self.joints_state.append(rospy.Subscriber('/shoulder_pan_joint_position_controller/state', JointControllerState, self.__shoulder_pan_listener))
        # self.joints_state.append(rospy.Subscriber('/shoulder_lift_joint_position_controller/state', JointControllerState, self.__shoulder_lift_listener))
        # self.joints_state.append(rospy.Subscriber('/elbow_joint_position_controller/state', JointControllerState, self.__elbow_listener))
        # self.joints_state.append(rospy.Subscriber('/wrist_1_joint_position_controller/state', JointControllerState, self.__wrist_1_listener))
        # self.joints_state.append(rospy.Subscriber('/wrist_2_joint_position_controller/state', JointControllerState, self.__wrist_2_listener))
        # self.joints_state.append(rospy.Subscriber('/wrist_3_joint_position_controller/state', JointControllerState, self.__wrist_3_listener))

        # Position and angle increment for simulation
        self.__incr = 0.01
        self.__incr_ = 0.01
        
# --------------------- Move the desired homogeneus transform -----------------
    def __move(self, T):
        q = self.__ur5.ikine_LM(T,q0 = self.__q)
        self.__q = q.q

        for i in range(6):
            self.__joints_com[i].publish(self.__q[i])
            self.__rate.sleep()
    
# ----------------------- Each move case (X, Y, Z, ROLL, PITCH, YAW) -----------------------    
    def move_x(self):
        T = self.__ur5.fkine(self.__q)
        print(T)
        T_ = SE3(self.__incr, 0, 0.0)
        T = T_ * T                      # The increment homogeneus matrix is pre-multiplied 
        
        self.__move(T)
    
    def move_x_(self):
        T = self.__ur5.fkine(self.__q)
        print(T)
        T_ = SE3(-self.__incr, 0, 0.0)
        T = T_ * T
        
        self.__move(T)
    
    def move_y(self):
        T = self.__ur5.fkine(self.__q)
        print(T)
        T_ = SE3(0, self.__incr, 0.0)
        T = T_ * T
        
        self.__move(T)
    
    def move_y_(self):
        T = self.__ur5.fkine(self.__q)
        print(T)
        T_ = SE3(0, -self.__incr, 0.0)
        T = T_ * T
        
        self.__move(T) 
    
    def move_z(self):
        T = self.__ur5.fkine(self.__q)
        print(T)
        T_ = SE3(0, 0, self.__incr)
        T = T_ * T
        
        self.__move(T)
    
    def move_z_(self):
        T = self.__ur5.fkine(self.__q)
        print(T)
        T_ = SE3(0, 0, -self.__incr)
        T = T_ * T
        
        self.__move(T)
    
    def rot_x(self):
        T = self.__ur5.fkine(self.__q)
        print(T)
        T_ = SE3.RPY(self.__incr_, 0, 0)
        T = T * T_
        
        self.__move(T)
    
    def rot_y(self):
        T = self.__ur5.fkine(self.__q)
        print(T)
        T_ = SE3.RPY(0, self.__incr_, 0)
        T = T * T_
        
        self.__move(T)
    
    def rot_z(self):
        T = self.__ur5.fkine(self.__q)
        print(T)
        T_ = SE3.RPY(0, 0, self.__incr_)
        T = T * T_
        
        self.__move(T)
        
# ---------------- Home position ----------------
    def home(self):    
        self.__q = self.__q0
        
        for i in range(5):
            self.__joints_com[i].publish(self.__q[i])
            self.__rate.sleep()

# ----------------- Callbacks for the joint controller state Subscribers ------------------
    def __shoulder_pan_listener(self,data):    
        self.__q[0] = data.set_point
    def __shoulder_lift_listener(self,data):
        self.__q[1] = data.set_point
    def __elbow_listener(self,data):
        self.__q[2] = data.set_point
    def __wrist_1_listener(self,data):
        self.__q[3] = data.set_point
    def __wrist_2_listener(self,data):
        self.__q[4] = data.set_point
    def __wrist_3_listener(self,data):
        self.__q[5] = data.set_point
        
# Declare controller
ur5 = Controller()

# Callback for the keyboard
def callback(tile):
    global ur5
    
    if str(tile) == "'x'":
        ur5.move_x()
        
    elif str(tile) == "'y'":
        ur5.move_y()
        
    elif str(tile) == "'z'":
        ur5.move_z()
        
    elif str(tile) == "'a'":
        ur5.move_x_()
        
    elif str(tile) == "'b'":
        ur5.move_y_()
        
    elif str(tile) == "'c'":
        ur5.move_z_()
        
    elif str(tile) == "'r'":
        ur5.rot_x()

    elif str(tile) == "'p'":
        ur5.rot_y()
    
    elif str(tile) == "'l'":
        ur5.rot_z()
    
    else:
        ur5.home()

# ------------------ Main --------------------
if __name__ == '__main__':
    with kb.Listener(callback) as listener:
	    listener.join()     