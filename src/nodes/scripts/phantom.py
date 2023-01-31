#! /usr/bin/python3

from control_msgs.msg import JointControllerState
from geometry_msgs.msg import PoseStamped, Pose, WrenchStamped
from sensor_msgs.msg import Joy, JointState
import rospy
import sys
import time
from spatialmath import *
import roboticstoolbox as rtb
from math import pi

# Button: sensor_msgs/Joy /arm/button1    /arm/button2

# Wrench: geometry_msgs/WrenchStamped   /arm/servo_cf
# Fuerza: y = 0.9
'''
x_limit: 0.09 - -0.07           * 2
y_limit: 0.2                    * 2
z_limit: 0.19  - -0.075         * 0.4 // *7
'''

'''
OBJETIVO
    El Phantom presenta 5 GDL pero solo 3 son actuables para el feedback de fuerza. Por ello, se mueve en XYZ con las coordenadas XYZ del Phantom
por defecto.
    Al presionar un botón, se cambia el modo de movimiento; las coordenadas RPY del robot se mueven con las coordenadas XYZ del Phantom.
    
    Al presionar el otro botón se cambia el modo de control de posición a velocidad o viceversa, con el mismo modus operandi respecto al modo de
movimiento.
'''

ur5 = rtb.DHRobot([
            rtb.RevoluteDH(d=0.1625, alpha=pi/2.0),
            rtb.RevoluteDH(a=-0.425),
            rtb.RevoluteDH(a = -0.3922),
            rtb.RevoluteDH(d = 0.1333, alpha=pi/2.0),
            rtb.RevoluteDH(d = 0.0997, alpha=-pi/2.0),
            rtb.RevoluteDH(d = 0.0996)
        ], name="UR5e")

ur5.base = SE3.RPY(0,0,pi)      # Rotate robot base so it matches Gazebo model
        


# Mensajes de la posición del robot y del wrench
pose = Pose()
wrench = WrenchStamped()

# Factor de escala de los movimientos 
scale_x = 2
scale_y = 2
scale_z1 = 3.5 # Diferente según está por encima o por debajo de 0
scale_z2 = 7

# Origen del robot
or_x = 0.5095
or_y = 0.1334
or_z = 0.7347

or_roll = 1.57225399
or_pitch = 1.07079575
or_yaw = -0.001661

# Posiciones previas para XYZ y RPY
#   Para al cambiar de XYZ a RPY y viceversa se guarden las XYZ o RPY segun convenga en el mensaje de pose
prev_x, prev_y, prev_z = or_x, or_y, or_z
prev_roll, prev_pitch, prev_yaw = or_roll , or_pitch, or_yaw

# Modos y Flags
xyz = True              # Flag para el modo de movimiento XYZ (FALSE = RPY)
change = True           # Flag para indicar cambio (a True para mandarlo al (0,0,0) al empezar)


# Posición articular
q = [0, -1.5, 1 , 0.0, 1.57, 0.0]

# Posición actual y previa del Phantom
prev_pose_phantom = Pose()
act_pose_phantom = Pose()

prev_pose_phantom.position.x = 0.0
prev_pose_phantom.position.y = 0.0
prev_pose_phantom.position.z = 0.0
prev_pose_phantom.orientation.x = 0.0
prev_pose_phantom.orientation.y = 0.0
prev_pose_phantom.orientation.z = 0.0

act_pose_phantom.position.x = -1.0
act_pose_phantom.position.y = -1.0
act_pose_phantom.position.z = -1.0
act_pose_phantom.orientation.x = -1.0
act_pose_phantom.orientation.y = -1.0
act_pose_phantom.orientation.z = -1.0



# Ganancia del feedback de fuerza
K = 80
limit = 0.001


# Tiempo para coger los mensajes del /joint_states
interval = 0.0
prev = time.time()


# Callback de la posicion cartesiana del robot
def cart_cb(data):
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    
    prev_x = data.position.x
    prev_y = data.position.y
    prev_z = data.position.z
    
    prev_roll = data.orientation.x
    prev_pitch = data.orientation.y
    prev_yaw = data.orientation.z

# Callback de las posiciones del Phantom
'''
Las coordenadas del Phantom son diferentes de las del robot --> 
  - x_robot = z_phantom
  - y_robot = x_phantom
  - z_robot = y_phantom
  
Se actúa sobre dos variables: 
  - pose: comando de posición al robot
  - prev_pose_phantom: posición previa del phantom.
     - .position: para XYZ
     - .orientation: para RPY (aunque ponga orientation, se usa para almacenar las posiciones XYZ del Phantom en el modo RPY)
  - act_pose_phantom: posición actual del Phantom
  
EXPLICACIÓN
    - 1. Almacena la posición actual en coordenadas del Phantom
    - 2. Si no hay un cambio ...
        - 2.1 ... se almacena la POSICIÓN del robot a partir de las coordenadas del Phantom (reescaladas y con el offset del origen)
            También se guarda la POSICIÓN actual de la posición del Phantom
            La orientación del robot es la anterior registrada
            
        - 2.2 ... se almacena la ORIENTACIÓN del robot a partir de las coordenadas del Phantom (reescaladas y con el offset del origen)
            También se guarda la ORIENTACIÓN actual de la posición del Phantom
            La posición del robot es la anterior registrada
'''
def cb(data):
    global pub, pose, scale_x, scale_y, scale_z1, scale_z2
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    global prev_pose_phantom, act_pose_phantom
    
    # 1. --
    act_pose_phantom.position = data.pose.position
    
    # 2. --
    if not change:
            # 2.1 --
            if xyz:
                pose.position.x = data.pose.position.z * scale_x + or_x
                pose.position.y = data.pose.position.x * scale_y + or_y
                
                if prev_z > 0:
                    pose.position.z = data.pose.position.y * scale_z1 + or_z
                else:
                    pose.position.z = data.pose.position.y * scale_z2 + or_z
                
                pose.orientation.x = prev_roll
                pose.orientation.y = prev_pitch
                pose.orientation.z = prev_yaw
                
                prev_pose_phantom.position = data.pose.position
                
            # 2.2 --    
            else:
                pose.position.x = prev_x
                pose.position.y = prev_y
                pose.position.z = prev_z

                pose.orientation.x = data.pose.position.z + or_roll
                pose.orientation.y = data.pose.position.x + or_pitch
                pose.orientation.z = data.pose.position.y + or_yaw
                
                prev_pose_phantom.orientation.x = data.pose.position.x
                prev_pose_phantom.orientation.y = data.pose.position.y
                prev_pose_phantom.orientation.z = data.pose.position.z
            

# Callbacks de los botones
def cb_bt1(data):
    global xyz, change
    
    if data.buttons[0] == 1:
        change = True
        
        print("XYZ - RPY")
        
        if xyz:
            xyz = False
        else:  
            xyz = True

def joint_state_cb(data):
    global q, prev, interval
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw

    if time.time() - prev > interval:         # Solo se ejecuta cuando pasa el intervalo
            
        prev = time.time()   

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

        
        T = ur5.fkine(q, order='xyz')

        trans = T.t
        eul = T.rpy(order='xyz')

        prev_x = trans[0]
        prev_y = trans[1]
        prev_z = trans[2]

        prev_roll = eul[0]
        prev_pitch = eul[1]
        prev_yaw = eul[2]



if __name__ == "__main__":
    
    if len(sys.argv) == 4:
        
        name = sys.argv[1]
        if name == "ur5_1":
            print("phantom " + name)

            # Nodo
            rospy.init_node(name + "_phantom_ctr")

            # Publishers: pose al robot, fuerza al Phantom y modo de movimiento para el robot
            pub = rospy.Publisher("/" + name + "/pose", Pose, queue_size=10)
            pub_f = rospy.Publisher("/" + name + "/servo_cf", WrenchStamped, queue_size=10)

            # Subscribers: posición del phantom, posición del robot, botones y cámaras del robot
            rospy.Subscriber("/" + name + "/measured_cp", PoseStamped,cb)
            rospy.Subscriber("/" + name + "/cart_pos", Pose, cart_cb)
            rospy.Subscriber("/" + name + "/button1", Joy, cb_bt1)
            rospy.Subscriber('/' + name + '/joint_states', JointState, joint_state_cb)

            # Rate
            r = rospy.Rate(20)

            # Bucle infinito
            '''
                - 1. El Phantom mantiene la posición sin moverse, compensando la gravedad (wrench obtenido experimentalmente)
                
                - 2. Si se pordujo un cambio (se presionó cualquiera de los botones para cambiar uno de los modos) ...
                    - 2.1 ... al cambio intentará volver a la anterior posición del Phantom registrada para la POSICIÓN del robot hasta cierto umbral
                    - 2.2 ... 

                - 3. Si no hubo cambio, se está funcionando normal, entonces envía las posiciones al robot
                    
                - 4. Se publican los Wrenches calculados según el caso
            '''

            while not rospy.is_shutdown():
                # 1 --
                if not change:
                    wrench.wrench.force.x = 0.0
                    wrench.wrench.force.y = 0.9
                    wrench.wrench.force.z = 0.0
                
                    
                # 2 --
                if change:
                    # print("change")
                    # 2.1 --
                    if xyz:
                        wrench.wrench.force.x = (prev_pose_phantom.position.x - act_pose_phantom.position.x)*K
                        wrench.wrench.force.y = (prev_pose_phantom.position.y - act_pose_phantom.position.y)*K
                        wrench.wrench.force.z = (prev_pose_phantom.position.z - act_pose_phantom.position.z)*K
                        
                        if (prev_pose_phantom.position.x - act_pose_phantom.position.x) < limit and (prev_pose_phantom.position.y - act_pose_phantom.position.y) < limit and (prev_pose_phantom.position.z - act_pose_phantom.position.z) < limit:
                            change = False
                            print("########################################")
                        
                    
                    # 2.2
                    else:
                        wrench.wrench.force.x = (prev_pose_phantom.orientation.x - act_pose_phantom.position.x)*K
                        wrench.wrench.force.y = (prev_pose_phantom.orientation.y - act_pose_phantom.position.y)*K
                        wrench.wrench.force.z = (prev_pose_phantom.orientation.z - act_pose_phantom.position.z)*K
                        
                        if (prev_pose_phantom.orientation.x - act_pose_phantom.position.x) < limit and (prev_pose_phantom.orientation.y - act_pose_phantom.position.y) < limit and (prev_pose_phantom.orientation.z - act_pose_phantom.position.z) < limit:
                            change = False

                            
                # 3 --   
                else:
                    print("pub")
                    pub.publish(pose)
                
                # 4 --
                pub_f.publish(wrench)    
                
                r.sleep()