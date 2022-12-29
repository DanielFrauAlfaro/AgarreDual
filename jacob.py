#! /usr/bin/python3


######## IMPORTANTE ############
# INSTALAR ROBOTIC TOOLBOX EN PYTHON: pip3 install roboticstoolbox-python



from spatialmath import *
import roboticstoolbox as rtb
from math import pi
import numpy as np
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
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
        
        self.__P = [0.4919, 0.1092, 0.6578]
        self.__O = [0.360399, 0.3611085, 0.607862, 0.60845]
        
        
        
        # ROS parameters: node, publishers and subscribers
        rospy.init_node("main_controller", anonymous=False)
        
        # rospy.Publisher('/pose', Pose, queue_size=10)
        rospy.Subscriber('/pose', Pose, self.__callback)
                
        self.__joints_com = []
        self.__joints_com.append(rospy.Publisher('/j1_vel', Float64, queue_size=10))
        self.__joints_com.append(rospy.Publisher('/j2_vel', Float64, queue_size=10))
        self.__joints_com.append(rospy.Publisher('/j3_vel', Float64, queue_size=10))
        self.__joints_com.append(rospy.Publisher('/j4_vel', Float64, queue_size=10))
        self.__joints_com.append(rospy.Publisher('/j5_vel', Float64, queue_size=10))
        self.__joints_com.append(rospy.Publisher('/j6_vel', Float64, queue_size=10))

        rospy.Subscriber("/joint_states", JointState, self.__states)

        # Position and angle increment for simulation
        self.__incr = np.array((0.01, 0, 0.0, 0, 0, 0))
    
    
    def control_loop(self):
                
        vel = [0,0,0,0,0,0]

        while not rospy.is_shutdown():
            
            J = self.__ur5.jacob0(self.__q)
            vel = np.linalg.inv(J) @ self.__incr

            for i in range(6):
                self.__joints_com[i].publish(vel[i])
                        
    
# -------------------- Callback for the haptic topic --------------------------
    def __callback(self, data):
        self.__incr[0] = data.position.x
        self.__incr[1] = data.position.y
        self.__incr[2] = data.position.z
        '''self.__incr[3] = data.orientation.x
        self.__incr[4] = data.orientation.y
        self.__incr[5] = data.orientation.z'''
        

# ----------------- Callbacks for the joint controller state Subscribers ------------------
    def __states(self, data):
        self.__q[0] = data.position[0]
        self.__q[1] = data.position[1]
        self.__q[2] = data.position[2]
        self.__q[3] = data.position[3]
        self.__q[4] = data.position[4]
        self.__q[5] = data.position[5]

        
        
# Controller object
ur5 = Controller()

# ------------------ Main --------------------
if __name__ == '__main__':
    ur5.control_loop()  