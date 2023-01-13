#! /usr/bin/python3


######## IMPORTANTE ############
# INSTALAR ROBOTIC TOOLBOX EN PYTHON: pip3 install roboticstoolbox-python

import  sys
from spatialmath import *
import roboticstoolbox as rtb
from math import pi
import numpy as np
import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from pynput import keyboard as kb
import math
import time
from pynput import keyboard


# Main class for the controller
class Controller():
    def __init__(self, name):
        # UR5 model in Robotic Toolbox
                
        self.__ur5 = rtb.DHRobot([
            rtb.RevoluteDH(d=0.1625, alpha=pi/2.0),
            rtb.RevoluteDH(a=-0.425),
            rtb.RevoluteDH(a = -0.3922),
            rtb.RevoluteDH(d = 0.1333, alpha=pi/2.0),
            rtb.RevoluteDH(d = 0.0997, alpha=-pi/2.0),
            rtb.RevoluteDH(d = 0.0996)
        ], name="UR5e")

        self.__ur5.base = SE3.RPY(0,0,pi)      # Rotate robot base so it matches Gazebo model
        
        # Joint position vector: actual and home
        self.__q = [0, -1.5, 1 , 0.0, 1.57, 0.0]
        self.__q0 = [0, -1.5, 1 , 0.0, 1.57, 0.0]
        
        # ROS parameters: node, publishers and subscribers
        rospy.init_node("main_controller", anonymous=False)
        
        # Subscriber para recibir posiciones y el modo de movimiento
        rospy.Subscriber('/' + name + '/pose', Pose, self.__callback)
        
        # Lista de publisher de las articulaciones
        self.__joints_com = []
        self.__joints_com.append(rospy.Publisher('/' + name + '/shoulder_pan_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/' + name + '/shoulder_lift_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/' + name + '/elbow_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/' + name + '/wrist_1_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/' + name + '/wrist_2_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/' + name + '/wrist_3_joint_position_controller/command', Float64, queue_size=100))

        rospy.Subscriber('/' + name + '/joint_states', JointState, self.__joint_state_cb)

        # Se publica la posición cartesiana
        self.__cart_pos = rospy.Publisher('/' + name + '/cart_pos', Pose, queue_size=10)
        
        # Origen
        self.T_or = self.__ur5.fkine(self.__q)
        self.__qp = [0, -1.5, 1 , 0.0, 1.57, 0.0]
        
        # Intervalos para ajustar la frecuencia de funcionamiento
        self.__interval = 0.0
        self.__prev = time.time()

        self.__interval2 = 0.2
        self.__prev2 = time.time()
        
        
# --------------------- Move the desired homogeneus transform -----------------
    def __move(self, T):
        q = self.__ur5.ikine_LMS(T,q0 = self.__q)       # Inversa: obtiene las posiciones articulares a través de la posición
        self.T_or = T                                   # Actualiza la T actual
        self.__qp = q.q
        
        pose = Pose()                                   # Se codifica el mensaje de la posición cartesiana actual
        
        trans = T.t
        eul = T.rpy(order='xyz')
        
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        
        pose.orientation.x = eul[0]
        pose.orientation.y = eul[1]
        pose.orientation.z = eul[2]
        
        self.__cart_pos.publish(pose)


    
# -------------------- Callback for the haptic topic --------------------------
    def __callback(self, data):
        if time.time() - self.__prev > self.__interval:         # Solo se ejecuta cuando pasa el intervalo
            
            self.__prev = time.time()                           # Actualiza el intervalo
            
            x = data.position.x                                 # Obtiene las posiciones
            y = data.position.y
            z = data.position.z
                
            roll = data.orientation.x
            pitch = data.orientation.y
            yaw = data.orientation.z
                          
            T = SE3(x, y, z)
            T_ = SE3.RPY(roll, pitch, yaw, order='xyz')
            
            T = T * T_
            
            self.__move(T)

            
        
# ------------------------ Bucle de control ------------------------
    def control_loop(self):
        
        self.T_or = self.__ur5.fkine(self.__q)
        rate = rospy.Rate(18)
        
        # El bucle solo ejecuta funciones si está en velocidad; necesita incrementar los valores
        while not rospy.is_shutdown(): 
            for i in range(6):                              # Se envían los valores
                self.__joints_com[i].publish(self.__qp[i])
            print("A")
            rate.sleep()
        
        
# ---------------- Home position ----------------
    def home(self, key):
        if key == keyboard.Key.esc:
            self.__q = [0, -1.5, 1 , 0.0, 1.57, 0.0]
            self.__qp = [0, -1.5, 1 , 0.0, 1.57, 0.0]
            for i in range(5):
                self.__joints_com[i].publish(self.__q0[i])

            self.T_or = self.__ur5.fkine([0, -1.5, 1 , 0.0, 1.57, 0.0])
                
        
    def __joint_state_cb(self, data):
        if time.time() - self.__prev2 > self.__interval2:         # Solo se ejecuta cuando pasa el intervalo
            
            self.__prev2 = time.time()   

            for i in range(len(data.name)):
                if data.name[i] == "shoulder_lift_joint":
                    
                    self.__q[0] = data.position[i]
                elif data.name[i] == "shoulder_pan_joint":
                    
                    self.__q[1] = data.position[i]
                elif data.name[i] == "elbow_joint":
                    
                    self.__q[2] = data.position[i]
                elif data.name[i] == "wrist_1_joint":
                    
                    self.__q[3] = data.position[i]
                elif data.name[i] == "wrist_2_joint":
                   
                    self.__q[4] = data.position[i]
                elif data.name[i] == "wrist_3_joint":
                    
                    self.__q[5] = data.position[i]
           
# ------------------ Main --------------------
if __name__ == '__main__':
    print(len(sys.argv))
    if len(sys.argv) == 4:

        name = sys.argv[1]

        ur5 = Controller(name)

        listener = keyboard.Listener(on_press=ur5.home)
        listener.start()

        ur5.control_loop()    