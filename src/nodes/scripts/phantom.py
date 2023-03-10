#! /usr/bin/python3

from std_msgs.msg import Float64, Int32, Float32MultiArray
from geometry_msgs.msg import PoseStamped, WrenchStamped, Pose
from sensor_msgs.msg import Joy
import rospy
import sys
import time
from math import pi

'''
x_limit: 0.09 - -0.07           * 2
y_limit: 0.2                    * 2
z_limit: 0.19  - -0.075         * 0.4 // *7

OBJETIVO
    El Phantom presenta 5 GDL pero solo 3 son actuables para el feedback de fuerza. Por ello, se mueve en XYZ con las coordenadas XYZ del Phantom
por defecto.

    Hay tres estados en los que se puede mover el robot:
        - Estado 0: Posición XYZ en el espacio
        - Estado 1: Control articular de la muñeca del robot
        - Estado 2: Control de la pinza
            · Estado 2.1: Control de los dedos
            · Estado 2.2: Control de la palma

    Las transiciones se realizan con los botones del Phantom, de manera que con el botón 1 se avanza el estado y con el botón
2 se retrocede.
    En el Estado 2 el botón 2 sirve para cambiar entre sub - estados, se manera que solo se puede avanzar de estado mediante el
botón 1.
'''

# Mensajes
pose = Pose()                       # Posición del extremo del robot
wrench = WrenchStamped()            # Wrench del Phantom

grip_pos = []

# Factor de escala de los movimientos 
scale_x = 2                         # Escala del movimiento cartesiano
scale_y = 2
scale_z = 3.5

scale_roll = 12                      # Escala del movimiento en ángulos de Euler
scale_pitch = 12
scale_yaw = 12

scale_grip_140 = 4.08               # Escala en el movimiento de las pinzas
scale_grip = 10
scale_grip_palm = 0.5

# Origen del robot
or_x = 0.5137                        # Origen del movimiento cartesiano
or_y = 0.1334                       
or_z = 0.4397

or_roll = -2.296 # 0.832 # 0.87                # Origen del movimiento articular de la muñeca del robot
or_pitch = 0.0 # 1.12                   # YXZ hacia abajo # YXZ # XYZ
or_yaw = -3.03 # 1.12 # -0.001661

or_3f = 0.0495

# Posiciones previas para XYZ y RPY
#   Para al cambiar de XYZ a RPY y viceversa se guarden las XYZ o RPY segun convenga en el mensaje de pose
prev_x, prev_y, prev_z = 0.516, 0.1334, 0.72
prev_roll, prev_pitch, prev_yaw = -2.296, 0.0, -3.03

prev_grip_pos = []


# Modos y Flags
change = True                       # Flag para indicar cambio (a True para mandarlo al (0,0,0) al empezar)
state = 0                           # Estado del movimiento 
state_3f = 0                        # Estado del movimiento de la pinza

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
K = 40                             # Ganancia en la posición
KD = 0.2

K_or = 10                          # Ganancia en la orientación
KD_or = 0.05

K_grip_2f = 10                     # Ganancia de la pinza de 2 dedos
KD_grip_2f = 0.05

K_grip_3f = 10                      # Ganancia de la pinza de 3 dedos
KD_grip_3f = 0.05

Ke = 70                             # Ganancia del feedback de fuerza en los cambios de modo
Kde = 0.2

# Umbral de detección para los cambios
limit = 0.005

# Nombre del robot y de la pinza
name = "ur5_2"
grip = "3f"

# Publicar modo en orientación
pub_orient = []

# Publisher para el cambio
pub_change = []

prev = time.time()
interval = 0.07

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
        
        - 2.3 ... se almacenan la posiciones del Phantom, correspondiente a la apertura y cierre de cada dedo de la pinza. 
            Se distingue entre dos casos: la pinza de 2 dedos y la de tres dedos en función del nombre asignado al robot.
'''
def cb(data):
    global pub, pose, scale_x, scale_y, scale_z, scale_roll, scale_pitch, scale_yaw
    global or_x, or_y, or_z, or_roll, or_pitch, or_yaw, or_3f
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    global prev_pose_phantom, act_pose_phantom
    global grip_pos
    global scale_grip_140, scale_grip
    global state_3f
    global grip
    global prev, interval

    if time.time() - prev > interval:
        prev = time.time()
        # 1. --
        act_pose_phantom.position = data.pose.position
        
        # 2. --
        if not change:
            # 2.1 --
            if state == 0:
                pose.position.x = data.pose.position.z * scale_x + or_x
                pose.position.y = data.pose.position.x * scale_y + or_y
                pose.position.z = data.pose.position.y * scale_z + or_z
                
                pose.orientation.x = prev_roll
                pose.orientation.y = prev_pitch
                pose.orientation.z = prev_yaw
                
                prev_pose_phantom.position = data.pose.position
                
            # 2.2 --    
            elif state == 1:
                pose.position.x = prev_x
                pose.position.y = prev_y
                pose.position.z = prev_z

                pose.orientation.x = data.pose.position.z * scale_roll + or_roll
                pose.orientation.y = data.pose.position.x * scale_pitch + or_pitch
                pose.orientation.z = data.pose.position.y * scale_yaw + or_yaw
                
                prev_pose_phantom.orientation = data.pose.position
                
            # 2.3 --
            elif state == 2:
                if grip == "2f":
                    grip_pos[0] = data.pose.position.y * scale_grip_140
                
                else:
                    if state_3f == 0:
                        grip_pos[0] = or_3f + data.pose.position.x * scale_grip
                        grip_pos[1] = or_3f + data.pose.position.y * scale_grip
                        grip_pos[2] = or_3f + data.pose.position.z * scale_grip

                    elif state_3f == 1:
                        grip_pos[3] = data.pose.position.y * scale_grip_palm


# Callbacks de los botones
def cb_bt1(data):
    global change, state, state_3f
    global pub_change

    if data.buttons[0] == 1:
        state = state + 1       # Incrementa el estado
        change = True
        state_3f = 0

        # Si se pasa del número de estados, vuelve al estado 0
        if state > 2:           
            state = 0
        
        pub_change[0].publish(-(state + 1))
        

# Callback del botón 2
def cb_bt2(data):
    global change
    global state, state_3f
    
    # Si está en el segundo estado sin cambio y se presiona el botón
    if state == 2 and not change:
        if data.buttons[0] == 1:
            # Cambia el subestado de la pinza
            state_3f = state_3f + 1
            change = True
            
            # Si se pasa del número de sub-estados, vuelve al 0
            if state_3f > 1:
                state_3f = 0

    # Si se está en cualquier otro, vuelve al anterior
    else:
        if data.buttons[0] == 1:
            state = state - 1
            change = True

            # Si se pasa por debajo de 0, vuelve al máximo
            if state < 0:
                state = 2

    pub_change[0].publish(-(state + 1))


# Estado del robot
def cart_pos(data):
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    global change

    # Si está en el estado 2, obtiene la posición del extremo
    
    if state == 0:
        prev_x = data.position.x
        prev_y = data.position.y
        prev_z = data.position.z

    # Si está en el estado 1 obtiene la orientación
    if state == 1:
        prev_roll = data.orientation.x
        prev_pitch = data.orientation.y
        prev_yaw = data.orientation.z

# Callback de la posición de la pinza
def grip_pos_cb(data):
    global grip
    global prev_grip_pos

    # En función de la pinza se asignan unos valores u otros
    for i in range(len(data.data)):
        prev_grip_pos[i] = data.data[i]


# Main
if __name__ == "__main__":

    if len(sys.argv) > 0:
        
        name = sys.argv[1]
        grip = ""

        if sys.argv[2] == '3f':
            grip = "3f"
            grip_pos = [or_3f, or_3f, or_3f, 0.0]
            prev_grip_pos = [or_3f, or_3f, or_3f, 0.0]
        elif sys.argv[2] == '2f_140':
            grip = "2f"
            grip_pos = [0.0]
            prev_grip_pos = [0.0]
        else:
            grip = ""
        

        # Nodo
        rospy.init_node(name + "_phantom_ctr")

        # Publishers: pose al robot, fuerza al Phantom y modo de movimiento para el robot
        pub = rospy.Publisher("/" + name + "/pose", Pose, queue_size=10)
        pub_f = rospy.Publisher("/" + name + "/servo_cf", WrenchStamped, queue_size=10)
        pub_change.append(rospy.Publisher("/" + name + "/change", Int32, queue_size=10))

        # Subscribers: posición del phantom, posición del robot, botones y cámaras del robot
        rospy.Subscriber("/" + name + "/measured_cp", PoseStamped,cb)
        rospy.Subscriber("/" + name + "/button1", Joy, cb_bt1)
        rospy.Subscriber("/" + name + "/button2", Joy, cb_bt2)
        rospy.Subscriber('/' + name + '/cart_pos', Pose, cart_pos)
        rospy.Subscriber("/" + name + "/grip_pos", Float32MultiArray, grip_pos_cb)
        
        pub_grip = []

        if grip != "":
            if grip == "2f":
                pub_grip.append(rospy.Publisher("/" + name + "/gripper/command", Float64, queue_size=10 ))

            else:
                pub_grip.append(rospy.Publisher("/" + name + "/finger_1_joint_1_controller/command", Float64, queue_size=10 ))
                pub_grip.append(rospy.Publisher("/" + name + "/finger_2_joint_1_controller/command", Float64, queue_size=10 ))
                pub_grip.append(rospy.Publisher("/" + name + "/finger_middle_joint_1_controller/command", Float64, queue_size=10 ))
                pub_grip.append(rospy.Publisher("/" + name + "/palm_finger_1_joint_controller/command", Float64, queue_size=10 ))
        
        t = time.time()

        # Rate
        r = rospy.Rate(15)

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
        
        ex = 0
        ey = 0
        ez = 0
        ex0 = 0
        ey0 = 0
        ez0 = 0
        
        while not rospy.is_shutdown():

            # 2 --
            if change:
                # 2.1 --
                if state == 0:
                    ex = (prev_pose_phantom.position.x - act_pose_phantom.position.x)
                    ey = (prev_pose_phantom.position.y - act_pose_phantom.position.y)
                    ez = (prev_pose_phantom.position.z - act_pose_phantom.position.z)

                # 2.2 --
                elif state == 1:
                    ex = (prev_pose_phantom.orientation.x - act_pose_phantom.position.x)
                    ey = (prev_pose_phantom.orientation.y - act_pose_phantom.position.y)
                    ez = (prev_pose_phantom.orientation.z - act_pose_phantom.position.z)

                # 2.3 --
                elif state == 2:
                    if grip == "2f":
                        ex = (0.0 - act_pose_phantom.position.x)
                        ey = (prev_grip_pos[0] / scale_grip_140 - act_pose_phantom.position.y)
                        ez = (0.0 - act_pose_phantom.position.z)

                    else:
                        if state_3f == 0:
                            ex = (prev_grip_pos[0] / scale_grip - act_pose_phantom.position.x)
                            ey = (prev_grip_pos[1] / scale_grip - act_pose_phantom.position.y)
                            ez = (prev_grip_pos[2] / scale_grip - act_pose_phantom.position.z)

                        elif state_3f == 1:
                            ex = (0.0 - act_pose_phantom.position.x)
                            ey = (0.0 - act_pose_phantom.position.y)
                            ez = (prev_grip_pos[3] / scale_grip_palm - act_pose_phantom.position.z)


                if abs(ex) < limit and abs(ey) < limit and abs(ez) < limit:
                    change = False
                    pub_change[0].publish((state + 1))

                        
            # 3 --   
            else:   
                if state == 0:
                    ex = (prev_y - pose.position.y)
                    ey = (prev_z - pose.position.z)
                    ez = (prev_x - pose.position.x)
                
                elif state == 1:
                    ex = (prev_pitch - pose.orientation.y)
                    
                    if prev_yaw < 0.0:
                        prev_yaw = 2*pi + prev_yaw
                    if pose.orientation.z < 0.0:
                        pose.orientation.z = 2*pi + pose.orientation.z

                    ey = (prev_yaw - pose.orientation.z)
                    ez = (prev_roll - pose.orientation.x)

                elif state == 2:
                    ex = (0 - act_pose_phantom.position.x) * 10
                    ez = (0 - act_pose_phantom.position.z) * 10
                    ex0 = (0 - act_pose_phantom.position.x) * 10
                    ey0 = (0 - act_pose_phantom.position.y) * 10
                    ez0 = (0 - act_pose_phantom.position.z) * 10


                    if grip == "2f":
                        ey = (prev_grip_pos[0] - grip_pos[0]) 
                            
                    else:
                        if state_3f == 0:
                            ex = (prev_grip_pos[0] - grip_pos[0])
                            ey = (prev_grip_pos[1] - grip_pos[1])
                            ez = (prev_grip_pos[2] - grip_pos[2])

                        elif state_3f == 1:
                            ey = (prev_grip_pos[3] - grip_pos[3])

                    if act_pose_phantom.position.x < 0:
                        ex = ex0
                    if act_pose_phantom.position.y < 0:
                        ey = ey0
                    if act_pose_phantom.position.z < 0:
                        ez = ez0

                    
# --------------------------------------------------------------------------------------------------

                # Publicar la posición
                pub.publish(pose)

                # Publicar el gripper
                for i in range(len(pub_grip)):
                    pub_grip[i].publish(grip_pos[i])


            # Cálculo de ganancias
            k = 0
            kd = 0

            if change:
                k = Ke
                kd = Kde

            elif state == 0:
                k = K
                kd = KD
            
            elif state == 1:
                k = K_or
                kd = KD_or
            
            elif state == 2:
                if grip == "2f":
                    k = K_grip_2f
                    kd = KD_grip_2f
            
                elif grip == "3f":
                    k = K_grip_3f
                    kd = KD_grip_3f 
            
            # Aplicar las fuerzas
            wrench.wrench.force.x = ex * k - ex / (time.time() - t) * kd
            wrench.wrench.force.y = ey * k - ey / (time.time() - t) * kd
            wrench.wrench.force.z = ez * k - ez / (time.time() - t) * kd

            # Fuerza constante hacia arriba
            if not change:
                wrench.wrench.force.y = wrench.wrench.force.y + 0.85

            # 4 --
            pub_f.publish(wrench)

            t = time.time()

            r.sleep()