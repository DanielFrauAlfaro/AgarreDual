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
        rospy.Subscriber('/pose', Pose, self.__callback)
        rospy.Subscriber('/move_type', Int32, self.__cb_mode)
        
        # Lista de publisher de las articulaciones
        self.__joints_com = []
        self.__joints_com.append(rospy.Publisher('/shoulder_pan_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/shoulder_lift_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/elbow_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/wrist_1_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/wrist_2_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/wrist_3_joint_position_controller/command', Float64, queue_size=100))
        
        # Subscribers de los joint_states
        rospy.Subscriber('/shoulder_pan_joint_position_controller/state', JointControllerState, self.__shoulder_pan_listener)
        rospy.Subscriber('/shoulder_lift_joint_position_controller/state', JointControllerState, self.__shoulder_lift_listener)
        rospy.Subscriber('/elbow_joint_position_controller/state', JointControllerState, self.__elbow_listener)
        rospy.Subscriber('/wrist_1_joint_position_controller/state', JointControllerState, self.__wrist_1_listener)
        rospy.Subscriber('/wrist_2_joint_position_controller/state', JointControllerState, self.__wrist_2_listener)
        rospy.Subscriber('/wrist_3_joint_position_controller/state', JointControllerState, self.__wrist_3_listener)

        # Se publica la posición cartesiana
        self.__cart_pos = rospy.Publisher('/cart_pos', Pose, queue_size=10)
        
        # Modo de funcionamiento
        self.__mode = "pos"
        
        # Incrementos
        self.__incr_vec = [0,0,0,0,0,0]
        
        # Origen
        self.T_or = self.__ur5.fkine(self.__q)
        self.__qp = [0, -1.5, 1 , 0.0, 1.57, 0.0]
        
        # Intervalos para ajustar la frecuencia de funcionamiento
        self.__interval = 0.1
        self.__prev = time.time()
        
        
# --------------------- Move the desired homogeneus transform -----------------
    def __move(self, T):
        q = self.__ur5.ikine_LMS(T,q0 = self.__q)       # Inversa: obtiene las posiciones articulares a través de la posición
        self.T_or = T                                   # Actualiza la T actual
        self.__qp = q.q
        
        for i in range(6):                              # Se envían los valores
            self.__joints_com[i].publish(self.__qp[i])
        
        
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


# --------------- Callback para cambiar el modo de funcionamiento ------------
    def __cb_mode(self, data):
        if data.data == 0:
            self.__mode = "pos"
        else:
            self.__mode = "vel"
    
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
            
            # Si está en posición obtiene transformada y llama a la función de movimiento
            if self.__mode == "pos":                
                T = SE3(x, y, z)
                T_ = SE3.RPY(roll, pitch, yaw, order='xyz')
                
                T = T * T_
                
                self.__move(T)
            
            # Si está en velocidad, asigna los incrementos
            else:
                self.__incr_vec = [x/5.0, y/5.0, z/5.0, roll/5.0, pitch/5.0, yaw/5.0]
            
            
        
# ------------------------ Bucle de control ------------------------
    def control_loop(self):
        
        self.T_or = self.__ur5.fkine(self.__q)
        rate = rospy.Rate(18)
        
        # El bucle solo ejecuta funciones si está en velocidad; necesita incrementar los valores
        while not rospy.is_shutdown(): 
            
            if self.__mode != "pos":
                
                # Se montan las matrices de transformación
                T = SE3(self.__incr_vec[0], self.__incr_vec[1], self.__incr_vec[2])
                T_rot = SE3.RPY(self.__incr_vec[3], self.__incr_vec[4], self.__incr_vec[5], order='xyz')
                
                T = T * T_rot           # Cuando los valores lleguen en euler
               
                if self.__incr_vec[3] == 0.0 and self.__incr_vec[4] == 0.0 and self.__incr_vec[5] == 0.0:
                    T = T * self.T_or
                
                elif self.__incr_vec[0] == 0.0 and self.__incr_vec[1] == 0.0 and self.__incr_vec[2] == 0.0:
                    T = self.T_or * T
                
                self.T_or = T
                
                self.__move(T)
            else:
                self.__incr_vec = [0,0,0,0,0,0]
                
                    
            rate.sleep()
        
        
# ---------------- Home position ----------------
    def home(self, key):
        if key == keyboard.Key.esc:
            self.__q = [0, -1.5, 1 , 0.0, 1.57, 0.0]
            self.__qp = [0, -1.5, 1 , 0.0, 1.57, 0.0]
            for i in range(5):
                self.__joints_com[i].publish(self.__q0[i])
                
            self.T_or = self.__ur5.fkine(self.__q0)


# ----------------- Callbacks for the joint controller state Subscribers ------------------
    def __shoulder_pan_listener(self,data):   
        self.__q[0] = data.process_value
        
    def __shoulder_lift_listener(self,data):
        self.__q[1] = data.process_value
        
    def __elbow_listener(self,data):
        self.__q[2] = data.process_value
        
    def __wrist_1_listener(self,data):
        self.__q[3] = data.process_value
        
    def __wrist_2_listener(self,data):
        self.__q[4] = data.process_value
        
    def __wrist_3_listener(self,data):
        self.__q[5] = data.process_value
        


# ------------------ Main --------------------
if __name__ == '__main__':

    ur5 = Controller("")

    listener = keyboard.Listener(on_press=ur5.home)
    listener.start()

    ur5.control_loop()    