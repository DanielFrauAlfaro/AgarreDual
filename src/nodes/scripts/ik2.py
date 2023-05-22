#! /usr/bin/python3

from pynput import keyboard
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import rospy
import numpy as np
import sys
from spatialmath import SE3
import roboticstoolbox as rtb
from math import pi
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal
from std_msgs.msg import Duration

# Client list
client = []

# Goal message
goal = FollowJointTrajectoryGoal()

# Initializes goal message
goal.trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

p = [JointTrajectoryPoint()]
p[0].positions = [0.0, -pi/2.0, pi/2.0, -pi/2.0, -pi/2.0, 0.0]
p[0].velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

p[0].time_from_start = rospy.Duration(2.5)

goal.trajectory.points = p
    
# List of state values and commands
q = [0.0, -pi/2.0, pi/2.0, -pi/2.0, -pi/2.0, 0.0]
qp = [0.0, -pi/2.0, pi/2.0, -pi/2.0, -pi/2.0, 0.0]


# UR5e model
ur5 = rtb.DHRobot([
            rtb.RevoluteDH(d=0.1625, alpha=pi/2.0),
            rtb.RevoluteDH(a=-0.425),
            rtb.RevoluteDH(a = -0.3922),
            rtb.RevoluteDH(d = 0.1333, alpha=pi/2.0),
            rtb.RevoluteDH(d = 0.0997, alpha=-pi/2.0),
            rtb.RevoluteDH(d = 0.0996)
        ], name="UR5e")
ur5.base = SE3.RPY(0,0,pi) 
ur5.tool = SE3(0.0, 0.0, 0.03)


# State callback
def joint_state_cb(data):
    global q

    end = [False, False, False, False, False, False]

    # Gets all the joint position values iterating the message
    for i in range(len(data.name)):
        if data.name[i] == "shoulder_lift_joint":
            q[1] = data.position[i]
            end[1] = True

        elif data.name[i] == "shoulder_pan_joint":
            q[0] = data.position[i]
            end[0] = True

        elif data.name[i] == "elbow_joint":
            q[2] = data.position[i]
            end[2] = True

        elif data.name[i] == "wrist_1_joint":
            q[3] = data.position[i]
            end[3] = True

        elif data.name[i] == "wrist_2_joint":
            q[4] = data.position[i]
            end[4] = True

        elif data.name[i] == "wrist_3_joint":
            q[5] = data.position[i]
            end[5] = True

        if not (False in end):
            break



# Median filter for each joint        
smooth = [[], [], [], [], [], []]
size_filt = 2

for i in range(6):
    for j in range(size_filt):
        smooth[i].append(q[i])


# Command pose callback
def cb(data):
    global ur5
    global client, goal
    global qp, q

    # Builds the homogenous matrix
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
            
    q_ = ur5.ikine_LMS(T, q0 = q)
    # qp = q_.q

    for i in range(6):                              
        smooth[i].pop(-1)
        smooth[i].insert(0, q_.q[i])
        q_.q[i] =  sum(smooth[i]) / size_filt


    # Sets the goal 
    goal.trajectory.points[0].time_from_start = rospy.Duration(0.01)
    goal.trajectory.points[0].positions = q_.q
    
    client[0].send_goal(goal)


# Home key callback
def home(key):
    global goal, client
    global q, qp

    if key == keyboard.Key.esc:
        
        q = [0.0, -pi/2.0, pi/2.0, -pi/2.0, -pi/2.0, 0.0]
        qp = [0.0, -pi/2.0, pi/2.0, -pi/2.0, -pi/2.0, 0.0]

        goal.trajectory.points[0].positions = qp
        goal.trajectory.points[0].time_from_start = rospy.Duration(2.5)
        client[0].send_goal(goal)


# ---- Main ----
if __name__ == "__main__":

    # Name from arguments
    name = sys.argv[1]
    
    # Node
    rospy.init_node(name + "_controller")

    # Client
    client.append(actionlib.SimpleActionClient("scaled_pos_joint_traj_controller/follow_joint_trajectory", FollowJointTrajectoryAction))
    
    # Waits fot the servers
    print(" -- Waiting for servers")
    client[0].wait_for_server()
    print(" --- Server found!!")

    # ---- Subscribers ----
    # Pose command topic
    rospy.Subscriber("/" + name + "/pose", Pose, cb)
    
    # State callback
    rospy.Subscriber('/' + name + '/joint_states', JointState, joint_state_cb)


    # Keyboard listener
    listener = keyboard.Listener(on_press=home)
    listener.start()


    # Spin the node
    rospy.spin()