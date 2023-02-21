#! /usr/bin/python3


import  sys
from spatialmath import SE3
import roboticstoolbox as rtb
from math import pi
import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import time
from pynput import keyboard


# Main class for the controller
class Controller():
    def __init__(self, name):

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
        
        # Joint position vector: actual and home
        self.__q = [0, -1.5, 1 , 0.0, 1.57, 0.0]
        self.__q0 = [0, -1.5, 1 , 0.0, 1.57, 0.0]
        
        # ROS parameters: node, publishers and subscribers
        rospy.init_node(name + "_controller", anonymous=False)
        
        # Subscriber para recibir posiciones y el modo de movimiento
        rospy.Subscriber('/' + name + '/pose', Pose, self.__callback)
        
        # Lista de publisher de las articulaciones
        self.__joints_com = []
        self.__joints_com.append(rospy.Publisher('/' + name + '/shoulder_pan_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/' + name + '/shoulder_lift_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/' + name + '/elbow_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/' + name + '/wrist_1_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/' + name + '/wrist_2_joint_position_controller/command', Float64, queue_size=100))
        self.__joints_com.append(rospy.Publisher('/' + name + '/wrist_3_joint_position_controller/command', Float64, queue_size=100))

        # Estado de las articulaciones
        rospy.Subscriber('/' + name + '/joint_states', JointState, self.__joint_state_cb)
        
        # Modo de orientación y flag de orientación
        rospy.Subscriber('/' + name + '/orientation', Bool, self.__orientation_cb)
        self.__orientation = False
        
        # Psociones a enviar por los topics
        self.__qp = [0, -1.5, 1 , 0.0, 1.57, 0.0]
        
        # Intervalos para ajustar la frecuencia de funcionamiento
        self.__interval = 0.0
        self.__prev = time.time()

        self.__interval2 = 0.2
        self.__prev2 = time.time()

        
# --------------------- Move the desired homogeneus transform -----------------
    def __move(self, T):
        q = self.__ur5.ikine_LMS(T,q0 = self.__q)       # Inversa: obtiene las posiciones articulares a través de la posición
        self.__qp = q.q
        
        for i in range(6):                              # Se envían los valores
            self.__joints_com[i].publish(self.__qp[i])


    
# -------------------- Callback for the haptic topic --------------------------
    def __callback(self, data):
        # Solo se ejecuta cuando pasa el intervalo
        if time.time() - self.__prev > self.__interval:
            # Actualiza el intervalo
            self.__prev = time.time()                           
            
            # Obtiene las posiciones
            x = data.position.x                                 
            y = data.position.y
            z = data.position.z
                
            roll = data.orientation.x
            pitch = data.orientation.y
            yaw = data.orientation.z

            # Si no está en orientación construye la matriz de transformación
            if True:           
                T = SE3(x, y, z)
                T_ = SE3.RPY(roll, pitch, yaw, order='yxz')
                
                T = T * T_
                
                # Envía a la función de movimiento
                self.__move(T)
            
            # Si se mueve en orientación, mueve directamente las articulaciones
            else:
                self.__joints_com[3].publish(roll)
                self.__joints_com[4].publish(pitch)
                self.__joints_com[5].publish(yaw)
        

    # Callback para el modo de orientación
    def __orientation_cb(self, data):
        self.__orientation = data
        

# ---------------- Home position ----------------
    def home(self, key):
        if key == keyboard.Key.esc:
            self.__q = [0, -1.5, 1 , 0.0, 1.57, 0.0]
            self.__qp = [0, -1.5, 1 , 0.0, 1.57, 0.0]
            for i in range(5):
                self.__joints_com[i].publish(self.__q0[i])                
        
# ---------------- Callback del estado de las articulaciones ----------------
    def __joint_state_cb(self, data):
        # Solo se ejecuta cuando pasa el intervalo
        if time.time() - self.__prev2 > self.__interval2:         
            
            self.__prev2 = time.time()   

            for i in range(len(data.name)):
                if data.name[i] == "shoulder_lift_joint":
                    self.__q[0] = data.position[i]

                elif data.name[i] == "shoulder_pan_joint":
                    self.__q[1] = data.position[i]

                elif data.name[i] == "elbow_joint":
                    self.__q[2] = data.position[i]

                elif data.name[i] == "wrist_1_joint":
                    self.__q[3] = data.position[i]

                elif data.name[i] == "wrist_2_joint":
                    self.__q[4] = data.position[i]

                elif data.name[i] == "wrist_3_joint":
                    self.__q[5] = data.position[i]
           
           
# ------------------ Main --------------------
if __name__ == '__main__':
    if len(sys.argv) == 4:

        name = sys.argv[1]
        ur5 = Controller(name)

        listener = keyboard.Listener(on_press=ur5.home)
        listener.start()

        rospy.spin()    