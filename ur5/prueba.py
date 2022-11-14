#! /usr/bin/python3

from spatialmath import *
import roboticstoolbox as rtb
from math import pi
import matplotlib as plt
import cv2 as cv
import numpy as np
import rospy
from std_msgs.msg import Float64

from pynput import keyboard as kb


class Controller():
    
    def __init__(self):
        self.ur5 = rtb.models.DH.UR5()
        self.q = [0, -1.5, 1 , 0.0, 0.0, 0.0]
        
        #rospy.init_node("main_controller", anonymous=False)
        
        #self.pub = []
        #self.pub.append(rospy.Publisher('/shoulder_pan_joint_position_controller/command', Float64, queue_size=10))
        #self.pub.append(rospy.Publisher('/shoulder_lift_joint_position_controller/command', Float64, queue_size=10))
        #self.pub.append(rospy.Publisher('/elbow_joint_position_controller/command', Float64, queue_size=10))
        #self.pub.append(rospy.Publisher('/wrist_1_joint_position_controller/command', Float64, queue_size=10))
        #self.pub.append(rospy.Publisher('/wrist_2_joint_position_controller/command', Float64, queue_size=10))
        #self.pub.append(rospy.Publisher('/wrist_3_joint_position_controller/command', Float64, queue_size=10))

        self.incr = 0.01
        
    def move_x(self):
        T = self.ur5.fkine(self.q)
        print(T)
        T_ = SE3(self.incr, 0, 0.0)
        T = T_ * T                  # Primero se pone el incremento
        
        q = self.ur5.ikine_LM(T,q0 = self.q)
        self.q = q.q

        rate = rospy.Rate(10)

        for i in range(6):
            self.pub[i].publish(self.q[i])
            rate.sleep()
    
    def move_x_(self):
        T = self.ur5.fkine(self.q)
        print(T)
        T_ = SE3(-self.incr, 0, 0.0)
        T = T_ * T
        
        q = self.ur5.ikine_LM(T,q0 = self.q)
        self.q = q.q

        rate = rospy.Rate(10)

        for i in range(6):
            self.pub[i].publish(self.q[i])
            rate.sleep()
    
    def move_y(self):
        T = self.ur5.fkine(self.q)
        print(T)
        T_ = SE3(0, self.incr, 0.0)
        T = T_ * T
        
        q = self.ur5.ikine_LM(T,q0 = self.q)
        self.q = q.q
        print(T)
        rate = rospy.Rate(10)
        
        for i in range(6):
            self.pub[i].publish(self.q[i])
            rate.sleep() 
    
    def move_y_(self):
        T = self.ur5.fkine(self.q)
        print(T)
        T_ = SE3(0, -self.incr, 0.0)
        T = T_ * T
        
        q = self.ur5.ikine_LM(T,q0 = self.q)
        self.q = q.q
        print(T)
        rate = rospy.Rate(10)
        
        for i in range(6):
            self.pub[i].publish(self.q[i])
            rate.sleep() 
    
    def move_z(self):
        T = self.ur5.fkine(self.q)
        print(T)
        T_ = SE3(0, 0, self.incr)
        T = T_ * T
        
        q = self.ur5.ikine_LM(T,q0 = self.q)
        self.q = q.q
        print(T)
        rate = rospy.Rate(10)

        for i in range(6):
            self.pub[i].publish(self.q[i])
            rate.sleep()
    
    def move_z_(self):
        T = self.ur5.fkine(self.q)
        print(T)
        T_ = SE3(0, 0, -self.incr)
        T = T_ * T
        
        q = self.ur5.ikine_LM(T,q0 = self.q)
        self.q = q.q
        print(T)
        rate = rospy.Rate(10)

        for i in range(6):
            self.pub[i].publish(self.q[i])
            rate.sleep()
    
    def home(self):    
        self.q = [pi, -1.5, 1 , 0.0, 0.0, 0.0]
        rate = rospy.Rate(10)
        for i in range(5):
            self.pub[i].publish(self.q[i])
            rate.sleep()

ur5 = Controller()

def callback(tecla):
    global ur5
    
    if str(tecla) == "'x'":
        ur5.move_x()
        
    elif str(tecla) == "'y'":
        ur5.move_y()
        
    elif str(tecla) == "'z'":
        ur5.move_z()
        
    elif str(tecla) == "'a'":
        ur5.move_x_()
        
    elif str(tecla) == "'b'":
        ur5.move_y_()
        
    elif str(tecla) == "'c'":
        ur5.move_z_()    
    else:
        ur5.home()
    
    m = ur5.ur5.manipulability(ur5.q, axes="trans")
        
if __name__ == '__main__':
    ur5.ur5.base.rotx(pi)
    with kb.Listener(callback) as escuchador:
	    escuchador.join()     