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


# Modelo del UR5e
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

# Posición articular
q = [0, -1.57, 1.57 , -1.57, -1.57, 0.0]
qd = [0, 0, 0, 0, 0, 0]
tau = [0, 0, 0, 0, 0, 0]

# Mensajes a enviar
p = Pose()
g = Float32MultiArray()
f = Wrench()

# Tiempo para coger los mensajes del /joint_states
interval = 0.0
prev = time.time()

# Nombres
name = "ur5_2"
grip = "2f_140"

# Posiciones previas de la pinza (de dos o de tres dedos)
prev_grip_140 = 0
prev_grip_3f_1, prev_grip_3f_2, prev_grip_3f_mid, prev_grip_3f_palm = 0, 0, 0, 0

grip_pos = []

# Publishers
pubs = []

# Estado de las articulaciones
def joint_state_cb(data):
    global q, qd, prev, interval
    global p, g, f, pubs
    global prev_grip_140
    global name, grip
    global grip_pos

    # Cuando pasa el intervalo de tiempo se ejecuta el feedback
    if time.time() - prev > interval:        
        
        # Obtiene los valores articulares del robot
        for i in range(len(data.name)):
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

        # Cinemática directa (CD)
        T = ur5.fkine(q, order='yxz')

        # Obtiene los valores de traslación y rotación en ángulos de Euler
        trans = T.t
        eul = T.rpy(order='yxz')

        # Construye el mensaja para la posición    
        p.position.x = trans[0]
        p.position.y = trans[1]
        p.position.z = trans[2]

        p.orientation.x = eul[0]
        p.orientation.y = eul[1]
        p.orientation.z = eul[2]

        # Crea el mensaje para el gripper
        g.data = grip_pos

        # Publica los mensajes
        pubs[0].publish(p)
        pubs[1].publish(g)

        # Actualiza el tiempo
        prev = time.time()


# Main
if __name__ == '__main__':

    if len(sys.argv) > 0:
        
        name = sys.argv[1]
        grip = sys.argv[2]

        # Nodo
        rospy.init_node(name + "_cart_pos")
        if grip == "2f_140":
            grip_pos = [0.0]
        elif grip == "3f":
            grip_pos = [0.0, 0.0, 0.0, 0.0]

        rospy.Subscriber('/' + name + '/joint_states', JointState, joint_state_cb)
        pubs.append(rospy.Publisher("/" + name + "/cart_pos", Pose, queue_size=10))
        
        if grip != "none":
            pubs.append(rospy.Publisher("/" + name + '/grip_pos', Float32MultiArray, queue_size=10))

        rospy.spin()