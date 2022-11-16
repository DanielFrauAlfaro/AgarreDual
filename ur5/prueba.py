#! /usr/bin/python3


######## IMPORTANTE ############
# INSTALAR ROBOTIC TOOLBOX EN PYTHON: pip3 install roboticstoolbox-python



from spatialmath import *
import roboticstoolbox as rtb
from math import pi
import numpy as np
import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from pynput import keyboard as kb
import math
import time
 
 # Transformar de cuaternios a ángulos de Euler
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

# Main class for the controller
class Controller():
    def __init__(self):
        # UR5 model in Robotic Toolbox
        self.__ur5 = rtb.models.DH.UR5()
        self.__ur5.base = SE3.RPY(0,0,pi)      # Rotate robot base so it matches Gazebo model
        
        # Joint position vector: actual and home
        self.__q = [0, -1.5, 1 , 0.0, 0.0, 0.0]
        self.__q0 = [0, -1.5, 1 , 0.0, 0.0, 0.0]
        
        # ROS parameters: node, publishers and subscribers
        rospy.init_node("main_controller", anonymous=False)
                
        self.__joints_com = []
        self.__joints_com.append(rospy.Publisher('/shoulder_pan_joint_position_controller/command', Float64, queue_size=1))
        self.__joints_com.append(rospy.Publisher('/shoulder_lift_joint_position_controller/command', Float64, queue_size=1))
        self.__joints_com.append(rospy.Publisher('/elbow_joint_position_controller/command', Float64, queue_size=1))
        self.__joints_com.append(rospy.Publisher('/wrist_1_joint_position_controller/command', Float64, queue_size=1))
        self.__joints_com.append(rospy.Publisher('/wrist_2_joint_position_controller/command', Float64, queue_size=1))
        self.__joints_com.append(rospy.Publisher('/wrist_3_joint_position_controller/command', Float64, queue_size=1))

        rospy.Subscriber('/shoulder_pan_joint_position_controller/state', JointControllerState, self.__shoulder_pan_listener)
        rospy.Subscriber('/shoulder_lift_joint_position_controller/state', JointControllerState, self.__shoulder_lift_listener)
        rospy.Subscriber('/elbow_joint_position_controller/state', JointControllerState, self.__elbow_listener)
        rospy.Subscriber('/wrist_1_joint_position_controller/state', JointControllerState, self.__wrist_1_listener)
        rospy.Subscriber('/wrist_2_joint_position_controller/state', JointControllerState, self.__wrist_2_listener)
        rospy.Subscriber('/wrist_3_joint_position_controller/state', JointControllerState, self.__wrist_3_listener)

        # Position and angle increment for simulation
        self.__incr = 0.005
        self.__incr_ = 0.05
    
        
# --------------------- Move the desired homogeneus transform -----------------
    def __move(self, T):
        q = self.__ur5.ikine_LMS(T,q0 = self.__q)
        
        # mask: vector de traslacion x,y,z y rotacion r,p,y
        
        for i in range(6):
            self.__joints_com[i].publish(q.q[i])

    
# -------------------- Callback for the haptic topic --------------------------
    def __callback(self, data):
        x = data.position.x
        y = data.position.y
        z = data.position.z
        
        x_ = data.orientation.x
        y_ = data.orientation.y
        z_ = data.orientation.z
        w  = data.orientation.w
        
        (roll, pitch, yaw) = euler_from_quaternion(x_, y_, z_, w)
        
        T = SE3(x, y, z)
        T_ = SE3.RPY(roll, pitch, yaw)
        
        T = T * T_
        
        self.__move(T)
    
# ----------------------- Each move case (X, Y, Z, ROLL, PITCH, YAW) -----------------------    
    def move_x(self):
        t = time.time()
        T = self.__ur5.fkine(self.__q)
        T_ = SE3(self.__incr, 0, 0.0)
        
        T = T_ * T                      # The increment homogeneus matrix is pre-multiplied 
        
        print(T)
        
        T_t = T.t
        T_eul = T.eul()
        
        
        self.__move(T)
        print(time.time() - t)
        
    def move_x_(self):
        T = self.__ur5.fkine(self.__q)
        T_ = SE3(-self.__incr, 0, 0.0)
        T = T_ * T
        
        print(T)

        self.__move(T)
    
    def move_y(self):
        T = self.__ur5.fkine(self.__q)
        T_ = SE3(0, self.__incr, 0.0)
        T = T_ * T
        
        print(T)

        self.__move(T)
    
    def move_y_(self):
        T = self.__ur5.fkine(self.__q)
        T_ = SE3(0, -self.__incr, 0.0)
        T = T_ * T
        
        print(T)

        self.__move(T) 
    
    def move_z(self):
        T = self.__ur5.fkine(self.__q)
        T_ = SE3(0, 0, self.__incr)
        T = T_ * T
        
        print(T)

        self.__move(T)
    
    def move_z_(self):
        T = self.__ur5.fkine(self.__q)
        T_ = SE3(0, 0, -self.__incr)
        T = T_ * T
        
        print(T)

        self.__move(T)
    
    def rot_x(self):
        T = self.__ur5.fkine(self.__q)
        T_ = SE3.RPY(self.__incr_, 0, 0)
        T = T * T_                          # In angle-movement, the increment is post-multiplied
        
        print(T)

        self.__move(T)
    
    def rot_x_(self):
        T = self.__ur5.fkine(self.__q)
        T_ = SE3.RPY(-self.__incr_, 0, 0)
        T = T * T_
        
        print(T)

        self.__move(T)
    
    def rot_y(self):
        T = self.__ur5.fkine(self.__q)
        T_ = SE3.RPY(0, self.__incr_, 0)
        T = T * T_
        
        print(T)

        self.__move(T)
    
    def rot_y_(self):
        T = self.__ur5.fkine(self.__q)
        T_ = SE3.RPY(0, -self.__incr_, 0)
        T = T * T_
        
        print(T)

        self.__move(T)
    
    def rot_z(self):
        T = self.__ur5.fkine(self.__q)
        T_ = SE3.RPY(0, 0, self.__incr_)
        T = T * T_
        
        print(T)

        self.__move(T)
        
    def rot_z_(self):
        T = self.__ur5.fkine(self.__q)
        T_ = SE3.RPY(0, 0, -self.__incr_)
        T = T * T_
        
        print(T)

        self.__move(T)

        
# ---------------- Home position ----------------
    def home(self):    
        for i in range(5):
            self.__joints_com[i].publish(self.__q0[i])


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
        
        
# Controller object
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
        
    elif str(tile) == "'m'":
        ur5.rot_x_()

    elif str(tile) == "'n'":
        ur5.rot_y_()
    
    elif str(tile) == "'ñ'":
        ur5.rot_z_()
    
    else:
        ur5.home()

# ------------------ Main --------------------
if __name__ == '__main__':
    with kb.Listener(callback) as listener:
	    listener.join()     