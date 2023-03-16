#! /usr/bin/python3

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, Wrench, WrenchStamped
from sensor_msgs.msg import JointState
import rospy
import numpy as np
import sys
import time
from spatialmath import SE3
import roboticstoolbox as rtb
from math import pi


# Messages
p = Pose()
g = Float32MultiArray()
f = Wrench()
grip_pos = []


# UR5e model
ur5 = rtb.DHRobot([
            rtb.RevoluteDH(d=0.1625, alpha=pi/2.0),
            rtb.RevoluteDH(a=-0.425),
            rtb.RevoluteDH(a = -0.3922),
            rtb.RevoluteDH(d = 0.1333, alpha=pi/2.0),
            rtb.RevoluteDH(d = 0.0997, alpha=-pi/2.0),
            rtb.RevoluteDH(d = 0.0996)
        ], name="UR5e")

ur5.base = SE3.RPY(0,0,pi)      # Rotate robot base so it matches Gazebo model
ur5.tool = SE3(0.0, 0.0, 0.03)


# Joint position and velocity values
q = [0, -1.57, 1.57 , -1.57, -1.57, 0.0]
qd = [0, 0, 0, 0, 0, 0]


# Time intervals
interval = 0.0
prev = time.time()

# Names
name = "ur5_2"
grip = "2f_140"


# Publishers
pubs = []

# Joint states callback
def joint_state_cb(data):
    global q, qd, prev, interval
    global p, g, f, pubs
    global name, grip
    global grip_pos

    # When a time interval has passed, obtains all values
    if time.time() - prev > interval:        
        
        # Gets robot joint values iterating the data in the topic
        for i in range(len(data.name)):

            # UR5e joints
            if data.name[i] == "shoulder_pan_joint":
                q[0] = data.position[i]
                qd[0] = data.velocity[i]

            elif data.name[i] == "shoulder_lift_joint":
                q[1] = data.position[i]
                qd[1] = data.velocity[i]

            elif data.name[i] == "elbow_joint":
                q[2] = data.position[i]
                qd[2] = data.velocity[i]

            elif data.name[i] == "wrist_1_joint":
                q[3] = data.position[i]
                qd[3] = data.velocity[i]

            elif data.name[i] == "wrist_2_joint":
                q[4] = data.position[i]
                qd[4] = data.velocity[i]

            elif data.name[i] == "wrist_3_joint":
                q[5] = data.position[i]
                qd[5] = data.velocity[i]
            

            # Gripper joints: 2f or 3f
            elif grip == "2f_140" and data.name[i] == "finger_joint":
                grip_pos[0] = data.position[i]

            elif grip == "3f":
                if data.name[i] == "gripper_finger_1_joint_1":
                    grip_pos[0]= data.position[i]

                elif data.name[i] == "gripper_finger_2_joint_1":
                    grip_pos[1] = data.position[i]

                elif data.name[i] == "gripper_finger_middle_joint_1":
                    grip_pos[2] = data.position[i]
                
                elif data.name[i] == "gripper_palm_finger_1_joint":
                    grip_pos[3] = data.position[i]

        # Forward kinematics (CD)
        T = ur5.fkine(q, order='yxz')


        # J = ur5.jacob0(q, T)
        # print(np.linalg.det(J))


        # Gets the translation and orientation Euler values
        trans = T.t
        eul = T.rpy(order='yxz')

        # Builds up the robot pose message    
        p.position.x = trans[0]
        p.position.y = trans[1]
        p.position.z = trans[2]

        p.orientation.x = eul[0]
        p.orientation.y = eul[1]
        p.orientation.z = eul[2]

        # Builds up the gripper message
        g.data = grip_pos

        # Publish
        pubs[0].publish(p)
        pubs[1].publish(g)

        # Updates the time 
        prev = time.time()


# Main
if __name__ == '__main__':

    if len(sys.argv) > 0:
        
        # Names
        name = sys.argv[1]
        grip = sys.argv[2]

        # Assigns lists length according to grip name
        if grip == "2f_140":
            grip_pos = [0.0]
        elif grip == "3f":
            grip_pos = [0.0, 0.0, 0.0, 0.0]

        # Node
        rospy.init_node(name + "_cart_pos")

        # ------ Publisher ------
        # Cartesian robot position
        pubs.append(rospy.Publisher("/" + name + "/cart_pos", Pose, queue_size=10))
        
        # If there is a gripper, creates a publisher for the joint gripper position
        if grip != "none":
            pubs.append(rospy.Publisher("/" + name + '/grip_pos', Float32MultiArray, queue_size=10))

        # ------ Subscriber ------
        # Joint states
        rospy.Subscriber('/' + name + '/joint_states', JointState, joint_state_cb)

        # Spin the execution
        rospy.spin()