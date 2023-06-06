#! /usr/bin/python3

from pynput import keyboard
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import rospy
import numpy as np
import sys
import time
from spatialmath import SE3
import roboticstoolbox as rtb
from math import pi
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal
from std_msgs.msg import Duration

rospy.init_node("controller")

client = actionlib.SimpleActionClient("ur5_2/trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

goal = FollowJointTrajectoryGoal()

goal.trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

p = [JointTrajectoryPoint()]
p[0].positions = [-pi, -pi/2.0, pi/2.0, -pi/2.0, -pi/2.0, 0.0]
p[0].velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

p[0].time_from_start = rospy.Duration(2.5)

goal.trajectory.points = p
    


print(" -- Waiting for servers")
client.wait_for_server()
print(" --- Server found!!")


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



def joint_state_cb(data):
    global q

    end = [False, False, False, False, False, False]

    # Gets all the joint position values iterating the message
    for i in range(len(data.name)):
        if data.name[i] == "shoulder_lift_joint":
            q[0] = data.position[i]
            end[0] = True

        elif data.name[i] == "shoulder_pan_joint":
            q[1] = data.position[i]
            end[1] = True

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

        if end == [True, True, True, True, True, True]:
            break


# Median filter for each joint        
smooth = [[], [], [], [], [], []]
size_filt = 6

for i in range(6):
    for j in range(size_filt):
        smooth[i].append(q[i])

def cb(data):
    global ur5
    global client, goal
    global qp, q

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
    qp = q_.q

    # if qp[0] > 0.0:
    #     qp[0] = ((pi - qp[0]) + pi) * -1

    for i in range(6):                              
        smooth[i].pop(-1)
        smooth[i].insert(0, qp[i])
        qp[i] =  sum(smooth[i]) / size_filt

    print(qp[0])

    goal.trajectory.points[0].time_from_start = rospy.Duration(0.01)
    goal.trajectory.points[0].positions = qp

    # qp[0] = qp[0] + pi

    

    print(qp[0])

    client.send_goal(goal)


def home(key):
    global goal, client
    global q, qp

    if key == keyboard.Key.esc:
        
        q = [0.0, -pi/2.0, pi/2.0, -pi/2.0, -pi/2.0, 0.0]
        qp = [0.0, -pi/2.0, pi/2.0, -pi/2.0, -pi/2.0, 0.0]
        
        # p = [JointTrajectoryPoint()]

        # p[0].positions = [pi, -pi/2.0, pi/2.0, -pi/2.0, -pi/2.0, 0.0]
        # p[0].time_from_start = rospy.Duration(1)

        # goal.trajectory.points = p

        goal.trajectory.points[0].positions = qp

        client.send_goal(goal)


if __name__ == "__main__":
    rospy.Subscriber("/ur5_2/pose", Pose, cb)
    rospy.Subscriber('/joint_states', JointState, joint_state_cb)

    

    listener = keyboard.Listener(on_press=home)
    listener.start()

    

    rospy.spin()