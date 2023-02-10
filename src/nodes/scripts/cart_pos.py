#! /usr/bin/python3


from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import rospy
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

# Posiciones previas para XYZ y RPY
#   Para al cambiar de XYZ a RPY y viceversa se guarden las XYZ o RPY segun convenga en el mensaje de pose
prev_x, prev_y, prev_z = 0.5095, 0.1334, 0.7347
prev_roll, prev_pitch, prev_yaw = 1.57225, 1.07, -0.00166

# Mensajes a enviar
p = Pose()
g = Pose()

# Tiempo para coger los mensajes del /joint_states
interval = 0.1
prev = time.time()

# Nombre
name = "ur5_2"

# Posiciones previas de la pinza (de dos o de tres dedos)
prev_grip_140 = 0
prev_grip_3f_1, prev_grip_3f_2, prev_grip_3f_mid, prev_grip_3f_palm = 0, 0, 0, 0

# Publishers
cart_pos = rospy.Publisher("/" + name + "/cart_pos", Pose, queue_size=10)
grip_pos = rospy.Publisher(name + '/grip_pos', Pose, queue_size=10)



# Estado de las articulaciones
def joint_state_cb(data):
    global q, prev, interval
    global p, cart_pos
    global prev_grip_140

    if time.time() - prev > interval:         # Solo se ejecuta cuando pasa el intervalo
        
        # Obtiene los valores articulares del robot
        for i in range(len(data.name)):
            if data.name[i] == "shoulder_pan_joint":
                q[0] = data.position[i]

            elif data.name[i] == "shoulder_lift_joint":
                q[1] = data.position[i]

            elif data.name[i] == "elbow_joint":
                q[2] = data.position[i]

            elif data.name[i] == "wrist_1_joint":
                q[3] = data.position[i]

            elif data.name[i] == "wrist_2_joint":
                q[4] = data.position[i]

            elif data.name[i] == "wrist_3_joint":
                q[5] = data.position[i]
            
            elif name == "ur5_1" and data.name[i] == "finger_joint":
                prev_grip_140 = data.position[i]

            elif name == "ur5_2":
                if data.name[i] == "gripper_finger_middle_joint_1":
                    prev_grip_3f_1 = data.position[i]
                
                elif data.name[i] == "gripper_finger_1_joint_1":
                    prev_grip_3f_2 = data.position[i]

                elif data.name[i] == "gripper_finger_2_joint_1":
                    prev_grip_3f_mid = data.position[i]

                elif data.name[i] == "palm_finger_1_joint":
                    prev_grip_3f_palm = data.position[i]
        
        # Cinemática directa (CD)
        T = ur5.fkine(q, order='xyz')

        # Obtiene los valores de traslación y rotación en ángulos de Euler
        trans = T.t
        eul = T.rpy(order='xyz')

        # Obtiene las traslaciones lineales y angulares
        
        p.position.x = trans[0]
        p.position.y = trans[1]
        p.position.z = trans[2]

        p.orientation.x = eul[0]
        p.orientation.y = eul[1]
        p.orientation.z = eul[2]

        cart_pos.publish(p)

        # Actualiza el tiempo
        prev = time.time()



if __name__ == '__main__':

    if len(sys.argv) > 0:
        
        name = sys.argv[1]
        grip = sys.argv[2]

        # Nodo
        rospy.init_node(name + "_cart_pos")

        rospy.Subscriber('/' + name + '/joint_states', JointState, joint_state_cb)
        cart_pos = rospy.Publisher("/" + name + "/cart_pos", Pose, queue_size=10)
        if grip != "none":
            grip_pos = rospy.Publisher("/" + name + '/grip_pos', Pose, queue_size=10)

        rospy.spin()