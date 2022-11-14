# -*- coding: utf-8 -*-

'''
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from controller_window import Ui_Form
'''
import copy
# ROS
import rospy
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
import moveit_commander
import numpy as np
from std_msgs.msg import Float64
from pynput import keyboard as kb

def get_quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


class Controller():

    def __init__(self,parent=None):
        # super(GUI, self).__init__(parent)
        # self.ui = Ui_Form()
        # self.ui.setupUi(self)
        
        self.incr = 0.008
        
        self.q0 = [0, -1.5, 1, 0, 0, 0]
        
        rospy.init_node("main_controller", anonymous=False)
        
        self.pub = []
        self.pub.append(rospy.Publisher('/shoulder_pan_joint_position_controller/command', Float64, queue_size=10))
        self.pub.append(rospy.Publisher('/shoulder_lift_joint_position_controller/command', Float64, queue_size=10))
        self.pub.append(rospy.Publisher('/elbow_joint_position_controller/command', Float64, queue_size=10))
        self.pub.append(rospy.Publisher('/wrist_1_joint_position_controller/command', Float64, queue_size=10))
        self.pub.append(rospy.Publisher('/wrist_2_joint_position_controller/command', Float64, queue_size=10))
        self.pub.append(rospy.Publisher('/wrist_3_joint_position_controller/command', Float64, queue_size=10))

        self.arm = moveit_commander.MoveGroupCommander("arm")

        self.rate = rospy.Rate(10)
        
    # --------------------------- BUCLE DE CONTROL -------------------------
    def show(self):
        self.test()
        
    # ----------------------------------------------------------------------
    def move(self,pose):
        pose_ = []
        pose_.append(pose)
        arm_current_pose = self.arm.get_current_pose()
        self.arm.clear_pose_targets()
        self.arm.set_goal_tolerance(0.01)
                
        (plan, fraction) = self.arm.compute_cartesian_path(pose_, 0.005, 0.0)
        joints = plan.joint_trajectory.points
        print(len(joints))
        if len(plan.joint_trajectory.points) > 1:
            for j in range(len(joints)):
                for i in range(6):
                    self.pub[i].publish(joints[j].positions[i])
                    self.rate.sleep()
    
    def move_x(self):
        arm_current_pose = self.arm.get_current_pose()

        waypoint1 = Pose()
        waypoint1.position.x = arm_current_pose.pose.position.x + self.incr
        waypoint1.position.y = arm_current_pose.pose.position.y
        waypoint1.position.z = arm_current_pose.pose.position.z

        waypoint1.orientation = arm_current_pose.pose.orientation
        
        self.move(waypoint1)
            
    def move_y(self):
        arm_current_pose = self.arm.get_current_pose()

        waypoint1 = Pose()
        waypoint1.position.x = arm_current_pose.pose.position.x
        waypoint1.position.y = arm_current_pose.pose.position.y + self.incr
        waypoint1.position.z = arm_current_pose.pose.position.z

        waypoint1.orientation = arm_current_pose.pose.orientation
        
        self.move(waypoint1)
              
    def move_z(self):
        arm_current_pose = self.arm.get_current_pose()

        waypoint1 = Pose()
        waypoint1.position.x = arm_current_pose.pose.position.x
        waypoint1.position.y = arm_current_pose.pose.position.y
        waypoint1.position.z = arm_current_pose.pose.position.z + self.incr

        waypoint1.orientation = arm_current_pose.pose.orientation
        
        self.move(waypoint1)
    
    def home(self):
        for i in range(6):
            self.pub[i].publish(self.q0[i])
            self.rate.sleep()
    
    def test(self):
        waypoints = []
        arm_current_pose = self.arm.get_current_pose()
        self.arm.clear_pose_targets()
        self.arm.set_goal_tolerance(0.01)

        waypoint1 = Pose()
        waypoint1.position.x = 0.5
        waypoint1.position.y = 0.5
        waypoint1.position.z = 0.5
        
        x,y,z,w = get_quaternion_from_euler(3.1415, 0, 0)
        
        waypoint1.orientation.x = x
        waypoint1.orientation.y = y
        waypoint1.orientation.z = z
        waypoint1.orientation.w = w
        waypoint1.orientation = arm_current_pose.pose.orientation
        waypoints.append(copy.deepcopy(waypoint1))
        print("start")
        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.05, 0.0)  # waypoints to follow  # eef_step
        print("end")
        # self.arm.execute(plan, wait=True)
        
        print((plan.joint_trajectory.points[-1].positions))
        joints = plan.joint_trajectory.points[-1].positions

        rate = rospy.Rate(10)

        for i in range(6):
            self.pub[i].publish(joints[i])
            rate.sleep()

scullion = Controller()

def callback(tecla):
    global scullion

    if str(tecla) == "'x'":
        scullion.move_x()
        
    elif str(tecla) == "'y'":
        scullion.move_y()
        
    elif str(tecla) == "'z'":
        scullion.move_z()
        
    else:
        scullion.home()

if __name__ == '__main__':
    with kb.Listener(callback) as escuchador:
	    escuchador.join() 