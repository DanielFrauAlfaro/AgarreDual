#! /usr/bin/python3

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, WrenchStamped
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


# Posición articular
q = [0, -1.5, 1 , 0.0, 1.57, 0.0]
qd = [0, 0, 0, 0, 0, 0]
tau = [0, 0, 0, 0, 0, 0]

# Posiciones previas para XYZ y RPY
#   Para al cambiar de XYZ a RPY y viceversa se guarden las XYZ o RPY segun convenga en el mensaje de pose
prev_x, prev_y, prev_z = 0.5095, 0.1334, 0.7347
prev_roll, prev_pitch, prev_yaw = 1.57225, 1.07, -0.00166

# Mensajes a enviar
p = Pose()
g = Float32MultiArray()

# Tiempo para coger los mensajes del /joint_states
interval = 0.2
prev = time.time()

# Nombre
name = "ur5_2"
grip = "2f_140"

# Posiciones previas de la pinza (de dos o de tres dedos)
prev_grip_140 = 0
prev_grip_3f_1, prev_grip_3f_2, prev_grip_3f_mid, prev_grip_3f_palm = 0, 0, 0, 0

# Publishers
pubs = []


def torque_cb_shoulder_pan(data):
    global tau

    tau[0] = data.wrench.torque.z

def torque_cb_shoulder_lift(data):
    global tau

    tau[1] = data.wrench.torque.y

def torque_cb_elbow(data):
    global tau

    tau[2] = data.wrench.torque.y

def torque_cb_wrist_1(data):
    global tau

    tau[3] = data.wrench.torque.y

def torque_cb_wrist_2(data):
    global tau

    tau[4] = data.wrench.torque.z

def torque_cb_wrist_3(data):
    global tau

    tau[5] = data.wrench.torque.y



# Estado de las articulaciones
def joint_state_cb(data):
    global q, qd, prev, interval
    global p, g, pubs
    global prev_grip_140
    global name, grip
    global tau

    if time.time() - prev > interval:         # Solo se ejecuta cuando pasa el intervalo
        
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
                prev_grip_140 = data.position[i]

            elif grip == "3f":
                if data.name[i] == "gripper_finger_middle_joint_1":
                    prev_grip_3f_1 = data.position[i]
                
                elif data.name[i] == "gripper_finger_1_joint_1":
                    prev_grip_3f_2 = data.position[i]

                elif data.name[i] == "gripper_finger_2_joint_1":
                    prev_grip_3f_mid = data.position[i]

                '''elif data.name[i] == "gripper_palm_finger_1_joint":
                    prev_grip_3f_palm = data.position[i]'''

        # Cinemática directa (CD)
        T = ur5.fkine(q, order='xyz')
        J = ur5.jacob0(q, T)

        # J_ = np.linalg.inv(J)
        J_ = J.transpose()
        f = np.matmul(J, tau)
        
        '''np.set_printoptions(precision=2)
        print (f)
        print("-------------------------------")
        print("")'''

        # Obtiene los valores de traslación y rotación en ángulos de Euler
        trans = T.t
        eul = T.rpy(order='xyz')

        # Obtiene las traslaciones lineales y angulares
        
        p.position.x = round(trans[0],3)
        p.position.y = round(trans[1],3)
        p.position.z = round(trans[2],3)

        p.orientation.x = round(eul[0], 3)
        p.orientation.y = round(eul[1], 3)
        p.orientation.z = round(eul[2], 3)

        if grip == "2f_140":
            g_aux = [prev_grip_140]

            g.data = g_aux
            
        elif grip == "3f":
            g_aux = [prev_grip_3f_1, prev_grip_3f_2, prev_grip_3f_mid, prev_grip_3f_palm]

            g.data = g_aux

        pubs[0].publish(p)
        pubs[1].publish(g)

        # Actualiza el tiempo
        prev = time.time()




if __name__ == '__main__':

    if len(sys.argv) > 0:
        
        name = sys.argv[1]
        grip = sys.argv[2]

        # Nodo
        rospy.init_node(name + "_cart_pos")

        rospy.Subscriber('/' + name + '/joint_states', JointState, joint_state_cb)

        rospy.Subscriber('/' + name + '/shoulder_pan_joint_torque_sensor', WrenchStamped, torque_cb_shoulder_pan)
        rospy.Subscriber('/' + name + '/shoulder_lift_joint_torque_sensor', WrenchStamped, torque_cb_shoulder_lift)
        rospy.Subscriber('/' + name + '/elbow_joint_torque_sensor', WrenchStamped, torque_cb_elbow)
        rospy.Subscriber('/' + name + '/wrist_1_joint_torque_sensor', WrenchStamped, torque_cb_wrist_1)
        rospy.Subscriber('/' + name + '/wrist_2_joint_torque_sensor', WrenchStamped, torque_cb_wrist_2)
        rospy.Subscriber('/' + name + '/wrist_3_joint_torque_sensor', WrenchStamped, torque_cb_wrist_3)
        

        

        pubs.append(rospy.Publisher("/" + name + "/cart_pos", Pose, queue_size=10))
        
        if grip != "none":
            pubs.append(rospy.Publisher("/" + name + '/grip_pos', Float32MultiArray, queue_size=10))

        rospy.spin()