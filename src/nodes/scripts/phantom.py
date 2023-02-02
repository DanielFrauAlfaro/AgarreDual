#! /usr/bin/python3

from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped, Pose, Twist
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
vel = Twist()
prev_vel = Twist()
grip_140 = Float64()

# Factor de escala de los movimientos 
scale_x = 2
scale_y = 2
scale_z1 = 3.5 # Diferente según está por encima o por debajo de 0
scale_z2 = 7

scale_grip_140 = 0.245

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

prev_x_v, prev_y_v, prev_z_v = 0, 0, 0
prev_roll_v, prev_pitch_v, prev_yaw_v = 0, 0, 0

prev_grip_140 = 0

# Modos y Flags
change = True           # Flag para indicar cambio (a True para mandarlo al (0,0,0) al empezar)
state = 0

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
Ke = 10
Kd = 0.1
limit = 0.001


# Tiempo para coger los mensajes del /joint_states
interval = 0.0
prev = time.time()


# Nombre del robot
name = ""

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
        
        - 2.3 ... se almacena la posición en el eje Z del Phantom, correspondiente a la apertura y cierre de la pinza. 
            Se distingue entre dos casos: la pinza de 2 dedos y la de tres dedos en función del nombre asignado al robot.
'''
def cb(data):
    global pub, pose, scale_x, scale_y, scale_z1, scale_z2
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    global prev_pose_phantom, act_pose_phantom
    global grip_140, scale_grip_140
    
    
    # 1. --
    act_pose_phantom.position = data.pose.position
    
    # 2. --
    if not change:
            # 2.1 --
            if state == 0:
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
            elif state == 1:
                pose.position.x = prev_x
                pose.position.y = prev_y
                pose.position.z = prev_z

                pose.orientation.x = data.pose.position.z + or_roll
                pose.orientation.y = data.pose.position.x + or_pitch
                pose.orientation.z = data.pose.position.y + or_yaw
                
                prev_pose_phantom.orientation.x = data.pose.position.x
                prev_pose_phantom.orientation.y = data.pose.position.y
                prev_pose_phantom.orientation.z = data.pose.position.z

            # 2.3 --
            elif state == 2:
                if name == "ur5_1":
                    grip_140 = data.pose.position.z * scale_grip_140
                
                else:
                    pass

# Callback de la velocidad del Phantom
def cb_v(data):
    global vel, prev_vel
    prev_vel = vel
    vel = data.twist

# Callbacks de los botones
def cb_bt1(data):
    global change
    
    if data.buttons[0] == 1:
        state = state + 1       # Incrementa el estado
        change = True 

        if state > 2:           # Si se pasa del número de estados, ...
            state = 0           # ... vuelve al estado 0 ...
        
def cb_bt2(data):
    global change
    
    pass

# Estado de las articulaciones
def joint_state_cb(data):
    global q, prev, interval
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    global prev_x_v, prev_y_v, prev_z_v
    global prev_roll_v, prev_pitch_v, prev_yaw_v
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

        
        # Cinemática directa (CD)
        T = ur5.fkine(q, order='xyz')

        # Obtiene los valores de traslación y rotación en ángulos de Euler
        trans = T.t
        eul = T.rpy(order='xyz')

        # Computa las velocidad cartesianas y angulares derivando el desplazamiento
        prev_x_v = (trans[0] - prev_x_v) / (time.time() - prev)
        prev_y_v = (trans[1] - prev_y_v) / (time.time() - prev)
        prev_z_v = (trans[2] - prev_z_v) / (time.time() - prev)

        prev_roll_v = (eul[0] - prev_roll_v) / (time.time() - prev)
        prev_pitch_v = (eul[1] - prev_pitch_v) / (time.time() - prev)
        prev_yaw_v = (eul[2] - prev_yaw_v) / (time.time() - prev)

        # Obtiene las traslaciones lineales y angulares
        prev_x = trans[0]
        prev_y = trans[1]
        prev_z = trans[2]

        prev_roll = eul[0]
        prev_pitch = eul[1]
        prev_yaw = eul[2]

        # Actualiza el tiempo
        prev = time.time()



if __name__ == "__main__":
    
    if len(sys.argv) == 4:
        
        name = sys.argv[1]

        # Nodo
        rospy.init_node(name + "_phantom_ctr")

        # Publishers: pose al robot, fuerza al Phantom y modo de movimiento para el robot
        pub = rospy.Publisher("/" + name + "/pose", Pose, queue_size=10)
        pub_f = rospy.Publisher("/" + name + "/servo_cf", WrenchStamped, queue_size=10)

        # Subscribers: posición del phantom, posición del robot, botones y cámaras del robot
        rospy.Subscriber("/" + name + "/measured_cp", PoseStamped,cb)
        rospy.Subscriber('/' + name + '/measured_cv', TwistStamped, cb_v)
        rospy.Subscriber("/" + name + "/cart_pos", Pose, cart_cb)
        rospy.Subscriber("/" + name + "/button1", Joy, cb_bt1)
        rospy.Subscriber("/" + name + "/button2", Joy, cb_bt2)
        rospy.Subscriber('/' + name + '/joint_states', JointState, joint_state_cb)
        
        if name == "ur5_1":
            pub_140 = rospy.Publisher("/" + name + "/gripper/command", Float64, queue_size=10 )

        else:
            pass
        
        t = time.time()

        # Rate
        r = rospy.Rate(20)

        # Bucle infinito
        '''
            - 1. El Phantom mantiene la posición sin moverse, compensando la gravedad (wrench obtenido experimentalmente)
            
            - 2. Si se pordujo un cambio (se presionó cualquiera de los botones para cambiar uno de los modos) ...
                - 2.1 ... al cambio intentará volver a la anterior posición del Phantom registrada para la POSICIÓN del robot hasta cierto umbral
                - 2.2 ... al cambio intentará vovler a la anterior posición del Phantom registrada para la ORIENTACIÓN del robot hasta cierto umbral
                - 2.3 ... al cambio intentará volver a la anterior posición  del Phantom registrada para la PINZA del robot hasta cierto umbral

            - 3. Si no hubo cambio, se está funcionando normal, entonces envía las posiciones al robot
                
            - 4. Se publican los Wrenches calculados según el caso
        '''
        
        while not rospy.is_shutdown():
            # 1 --
            if not change:
                '''wrench.wrench.force.x = 0.0
                wrench.wrench.force.y = 0.9
                wrench.wrench.force.z = 0.0'''

                if state == 0:
                    ex = (prev_x - pose.position.x)
                    ey = (prev_y - pose.position.y)
                    ez = (prev_z - pose.position.z)

                    wrench.wrench.force.x = ex * Ke - ex / (time.time() - t) * Kd
                    wrench.wrench.force.y = ey * Ke - ey / (time.time() - t) * Kd
                    wrench.wrench.force.z = ez * Ke - ez / (time.time() - t) * Kd

                elif state == 1:
                    ex = (prev_roll - pose.orientation.x)
                    ey = (prev_pitch - pose.orientation.y)
                    ez = (prev_yaw - pose.orientation.z)

                    wrench.wrench.force.x = ex * Ke - ex / (time.time() - t) * Kd
                    wrench.wrench.force.y = ey * Ke - ey / (time.time() - t) * Kd
                    wrench.wrench.force.z = ez * Ke - ez / (time.time() - t) * Kd

                elif state == 2:
                    if name == "ur5_1":
                        ex = (0 - act_pose_phantom.position.x)
                        ey = (0 - act_pose_phantom.position.y)
                        ez = (prev_grip_140 - grip_140)      
                        ez0 = (0 - act_pose_phantom.position.z)

                        wrench.wrench.force.x = ex * Ke - ex / (time.time() - t) * Kd
                        wrench.wrench.force.y = ey * Ke - ey / (time.time() - t) * Kd
                        wrench.wrench.force.z = ez * Ke - ez / (time.time() - t) * Kd

                        if act_pose_phantom.position.z < 0:
                            wrench.wrench.force.z = ez0 * Ke - ez0 / (time.time() - t) * Kd
            
                
            # 2 --
            if change:
                # 2.1 --
                if state == 0:
                    ex = (prev_pose_phantom.position.x - act_pose_phantom.position.x)
                    ey = (prev_pose_phantom.position.y - act_pose_phantom.position.y)
                    ez = (prev_pose_phantom.position.z - act_pose_phantom.position.z)

                    wrench.wrench.force.x = ex * K - ex / (time.time() - t) * Kd
                    wrench.wrench.force.y = ey * K - ey / (time.time() - t) * Kd
                    wrench.wrench.force.z = ez * K - ez / (time.time() - t) * Kd
                    
                    if ex < limit and ey < limit and ez < limit:
                        change = False
                    
                
                # 2.2 --
                elif state == 1:
                    ex = (prev_pose_phantom.orientation.x - act_pose_phantom.position.x)
                    ey = (prev_pose_phantom.orientation.y - act_pose_phantom.position.y)
                    ez = (prev_pose_phantom.orientation.z - act_pose_phantom.position.z)

                    wrench.wrench.force.x = ex * K - ex / (time.time() - t) * Kd
                    wrench.wrench.force.y = ey * K - ey / (time.time() - t) * Kd
                    wrench.wrench.force.z = ez * K - ez / (time.time() - t) * Kd
                    
                    if ex < limit and ey < limit and ez < limit:
                        change = False


                # 2.3 --
                elif state == 0:
                    ex = (0.0 - act_pose_phantom.position.x)
                    ey = (0.0 - act_pose_phantom.position.y)
                    ez = (prev_grip_140 / scale_grip_140 - act_pose_phantom.position.z)

                    wrench.wrench.force.x = ex * K - ex / (time.time() - t) * Kd
                    wrench.wrench.force.y = ey * K - ey / (time.time() - t) * Kd
                    wrench.wrench.force.z = ez * K - ez / (time.time() - t) * Kd
                    
                    if ex < limit and ey < limit and ez < limit:
                        change = False


                        
            # 3 --   
            else:
                if state != 2:
                    pub.publish(pose)

                else:
                    pub_140.publish(grip_140)
            
            # 4 --
            pub_f.publish(wrench)    
            
            t = time.time()
            r.sleep()