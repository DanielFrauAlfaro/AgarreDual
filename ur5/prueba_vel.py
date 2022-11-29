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
from geometry_msgs.msg import Pose
from pynput import keyboard as kb
import math
import time

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]
 
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
        self.__q = [0, -1.5, 1 , 0.0, 1.57, 0.0]
        self.__q0 = [0, -1.5, 1 , 0.0, 1.57, 0.0]
        
        self.__P = [0.4919, 0.1092, 0.6578]
        self.__O = [0.360399, 0.3611085, 0.607862, 0.60845]
        
        
        
        # ROS parameters: node, publishers and subscribers
        rospy.init_node("main_controller", anonymous=False)
        
        # rospy.Publisher('/pose', Pose, queue_size=10)
        rospy.Subscriber('/pose', Pose, self.__callback)
                
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
        self.__incr = [0,0,0,0,0,0]
    
    
    def control_loop(self):
        T_or = SE3(self.__P[0], self.__P[1], self.__P[2])
        
        (roll, pitch, yaw) = euler_from_quaternion(self.__O[0], self.__O[1], self.__O[2], self.__O[3])       
         
        T_ = SE3.RPY(roll, pitch, yaw)
        
        T_or = T_or * T_
        
        incr = [0,0,0,0,0,0]
        
        while not rospy.is_shutdown():
            
            for i in range(6):    
                incr[i] = self.__incr[i] / 5.0
            
            T = SE3(incr[0], incr[1], incr[2])
            T_ = SE3.RPY(incr[3], incr[4], incr[5])
            
            T = T * T_or
            T_or = T
            
            self.__move(T)
            
    
# --------------------- Move the desired homogeneus transform -----------------
    def __move(self, T):
        q = self.__ur5.ikine_LMS(T,q0 = self.__q)
        
        for i in range(6):
            self.__joints_com[i].publish(q.q[i])

    
# -------------------- Callback for the haptic topic --------------------------
    def __callback(self, data):
        self.__incr[0] = data.position.x
        self.__incr[1] = data.position.y
        self.__incr[2] = data.position.z
        
        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        w  = data.orientation.w
        
        (roll, pitch, yaw) = euler_from_quaternion(x, y, z, w)
        
        self.__incr[3] = roll
        self.__incr[4] = pitch
        self.__incr[5] = yaw
        



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
        
        
# Controller object
ur5 = Controller()

# ------------------ Main --------------------
if __name__ == '__main__':
    ur5.control_loop()  