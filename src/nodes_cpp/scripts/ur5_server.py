#! /usr/bin/python3

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import rospy
import numpy as np
import sys
import time
from spatialmath import SE3
import roboticstoolbox as rtb
from math import pi
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from pynput import keyboard


# Main class for the controller
class Controller():
    def __init__(self, name, grip):

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
        self.__ur5.tool = SE3(0.0, 0.0, 0.03)
        
        # Joint position vector: current, reference and home
        self.__q = [0, -1.57, 1.57 , -1.57, -1.57, 0.0]
        self.__qp = [0, -1.57, 1.57 , -1.57, -1.57, 0.0]
        self.__q0 = [0, -1.57, 1.57 , -1.57, -1.57, 0.0]
        
        # Node
        rospy.init_node("controller", anonymous=False)
        
        
        # ------ Publishers ------
        # List of UR5e joint publishers
        self.__joints_com = []
        self.__joints_com.append(rospy.Publisher('/' + name + '/arm_controller/command', Float64MultiArray, queue_size=1))
        
        # List of gripper joint publishers according to the gripper type
        self.grip_pub = []
        if grip != "":
            if grip == "2f_140":
                self.grip_pub.append(rospy.Publisher("/" + name + "/gripper/command", Float64, queue_size=10 ))

            else:
                self.grip_pub.append(rospy.Publisher("/" + name + "/gripper_1_controller/command", Float64MultiArray, queue_size=10 ))
                self.grip_pub.append(rospy.Publisher("/" + name + "/gripper_2_controller/command", Float64MultiArray, queue_size=10 ))
                self.grip_pub.append(rospy.Publisher("/" + name + "/gripper_middle_controller/command", Float64MultiArray, queue_size=10 ))
        
        self.__pub_f = (rospy.Publisher("/" + name + "/cart_pos", Pose, queue_size=10))

        # ------ Subscribers ------
        # Reference cartesian pose
        rospy.Subscriber('pose', Pose, self.__callback)

        # Joint states
        rospy.Subscriber('joint_states', JointState, self.__joint_state_cb)
        
        # Median filter for each joint        
        self.__smooth = [[], [], [], [], [], []]
        self.__size_filt = 3

        for i in range(6):
            for j in range(self.__size_filt):
                self.__smooth[i].append(self.__q[i])

        # Execution time intervals
        self.__interval = 0.0
        self.__prev = time.time()

        self.__interval2 = 0.0
        self.__prev2 = time.time()

        # Messages
        self.__p = Pose()

        
# --------------------- Move the desired homogeneus transform -----------------
    def __move(self, T):
        # Computes inverse kinematics
        q = self.__ur5.ikine_LMS(T,q0 = self.__q)       
        self.__qp = q.q
        
        msg = Float64MultiArray()

        # Applies median filter and publishes the joint values
        for i in range(6):                              
            self.__smooth[i].pop(-1)
            self.__smooth[i].insert(0, self.__qp[i])
            self.__qp[i] =  sum(self.__smooth[i]) / self.__size_filt

        msg.data = self.__qp
        self.__joints_com[0].publish(msg)    


    
# -------------------- Callback for the pose topic --------------------------
    def __callback(self, data):
        # When the interval has passed
        if time.time() - self.__prev > self.__interval:
            
            # Updates the time
            self.__prev = time.time()                           
            
            # Gets the message values
            x = data.position.x                                 
            y = data.position.y
            z = data.position.z
                
            roll = data.orientation.x
            pitch = data.orientation.y
            yaw = data.orientation.z
          
            # Builds the homogeneus matrix
            T = SE3(x, y, z)
            T_ = SE3.RPY(roll, pitch, yaw, order='yxz')
            
            T = T * T_
            
            # Calls the movement function
            self.__move(T)
            

# ---------------- Home position ----------------
    def home(self, key):
        if key == keyboard.Key.esc:
            self.__q = [0, -1.5, 1 , 0.0, 1.57, 0.0]
            self.__qp = [0, -1.5, 1 , 0.0, 1.57, 0.0]
            
            msg = Float64MultiArray()
            msg.data = self.__q0

            self.__joints_com[0].publish(msg) 

            msg_g = Float64MultiArray()
            msg_g.data = [0.0, 0.0, 0.0]

            for i in range(len(self.grip_pub)):
                self.grip_pub[i].publish(msg_g) 
                           
        
# ---------------- Callback del estado de las articulaciones ----------------
    def __joint_state_cb(self, data):
        # When the interval has passed
        if time.time() - self.__prev2 > self.__interval2:         
            
            # Updates the time
            self.__prev2 = time.time()   

            end = [False, False, False, False, False, False]

            # Gets all the joint position values iterating the message
            for i in range(len(data.name)):
                if data.name[i] == "shoulder_lift_joint":
                    self.__q[0] = data.position[i]
                    end[0] = True

                elif data.name[i] == "shoulder_pan_joint":
                    self.__q[1] = data.position[i]
                    end[1] = True

                elif data.name[i] == "elbow_joint":
                    self.__q[2] = data.position[i]
                    end[2] = True

                elif data.name[i] == "wrist_1_joint":
                    self.__q[3] = data.position[i]
                    end[3] = True

                elif data.name[i] == "wrist_2_joint":
                    self.__q[4] = data.position[i]
                    end[4] = True

                elif data.name[i] == "wrist_3_joint":
                    self.__q[5] = data.position[i]
                    end[5] = True

                if end == [True, True, True, True, True, True]:
                    break
           
    # Joint states callback
    def forward(self):    
        # Forward kinematics (CD)
        T = self.__ur5.fkine(self.__q, order='yxz')

        # Gets the translation and orientation Euler values
        trans = T.t
        eul = T.rpy(order='yxz')

        # Builds up the robot pose message    
        self.__p.position.x = trans[0]
        self.__p.position.y = trans[1]
        self.__p.position.z = trans[2]

        self.__p.orientation.x = eul[0]
        self.__p.orientation.y = eul[1]
        self.__p.orientation.z = eul[2]

        self.__pub_f.publish(self.__p)


# ---- Main ----
if __name__ == '__main__':

    if len(sys.argv) > 0:
        
        # Names
        name = sys.argv[1]
        grip = sys.argv[2]

        ur5 = Controller(name, grip)

        listener = keyboard.Listener(on_press=ur5.home)
        listener.start()

        r = rospy.Rate(18)

        while not rospy.is_shutdown():
            
            ur5.forward()

            r.sleep()