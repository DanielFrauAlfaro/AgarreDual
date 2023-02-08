#! /usr/bin/python3

from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped, Pose
from sensor_msgs.msg import Joy
import rospy
import sys
import time
import nodes.msg

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

# Mensajes de la posición del robot y del wrench
pose = Pose()
wrench = WrenchStamped()

grip_140 = Float64()

grip_3f_1 = Float64()
grip_3f_2 = Float64()
grip_3f_mid = Float64()
grip_3f_palm = Float64()

# Factor de escala de los movimientos 
scale_x = 2
scale_y = 2
scale_z1 = 3.5 # Diferente según está por encima o por debajo de 0
scale_z2 = 7

scale_grip_140 = 4.08
scale_grip_3f = 5
scale_grip_3f_palm = 0.5

# Origen del robot
or_x = 0.516
or_y = 0.1334
or_z = 0.72

or_roll = 0.87
or_pitch = 1.12
or_yaw = -0.001661

# Posiciones previas para XYZ y RPY
#   Para al cambiar de XYZ a RPY y viceversa se guarden las XYZ o RPY segun convenga en el mensaje de pose
prev_x, prev_y, prev_z = 0.5095, 0.1334, 0.7347
prev_roll, prev_pitch, prev_yaw = 1.57225, 1.07, -0.00166

prev_grip_140 = 0
prev_grip_3f_1, prev_grip_3f_2, prev_grip_3f_mid, prev_grip_3f_palm = 0, 0, 0, 0

# Modos y Flags
change = True           # Flag para indicar cambio (a True para mandarlo al (0,0,0) al empezar)
state = 0
state_3f = 0

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
K = 0.00001
KD = 0.0

Ke = 50
Kde = 0.1

limit = 0.01


# Tiempo para coger los mensajes del /joint_states
interval = 0.0
prev = time.time()

# Nombre del robot
name = ""

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
    global act_pose_phantom, prev_pose_phantom, pose
    global grip_140, grip_3f_1, grip_3f_2, grip_3f_mid, grip_3f_palm

    act_pose_phantom = data.poses[0]
    pose = data.poses[1]
    grip_140 = data.g140
    grip_3f_1 = data.g3f_1
    grip_3f_2 = data.g3f_2
    grip_3f_mid = data.g3f_m
    grip_3f_palm = data.g3f_p

    if state == 0:
        prev_pose_phantom.position = data.poses[1].position

    elif state == 1:
        prev_pose_phantom.orientation = data.poses[1].orientation




# Callbacks de los botones
def cb_bt1(data):
    global change, state
    
    if data.buttons[0] == 1:
        state = state + 1       # Incrementa el estado
        change = True 

        if state > 2:           # Si se pasa del número de estados, ...
            state = 0           # ... vuelve al estado 0 ...
        
def cb_bt2(data):
    global change
    global state, state_3f
    
    if state == 2:
        if data.buttons[0] == 1:
            state_3f = state_3f + 1
            change = True

            if state_3f > 3:
                state_3f = 0


# Estado del robot
def cart_pos(data):
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw

    if state == 0:
        prev_x = data.position.x
        prev_y = data.position.y
        prev_z = data.position.z

    elif state == 1:
        prev_roll = data.orientation.x
        prev_pitch = data.orientation.y
        prev_yaw = data.orientation.z


if __name__ == "__main__":
    print(sys.argv)

    if len(sys.argv) > 0:
        
        # name = sys.argv[1]
        name = "ur5_2"
        name_p = "arm"

        # Nodo
        rospy.init_node(name + "_phantom_force_ctr")

        # Publishers: pose al robot, fuerza al Phantom y modo de movimiento para el robot
        pub_f = rospy.Publisher("/" + name_p + "/servo_cf", WrenchStamped, queue_size=10)

        # Subscribers: posición del phantom, posición del robot, botones y cámaras del robot
        rospy.Subscriber("/" + name_p + "/cmp_pose", nodes.msg.Poses,cb)

        rospy.Subscriber("/" + name_p + "/button1", Joy, cb_bt1)
        rospy.Subscriber("/" + name_p + "/button2", Joy, cb_bt2)
        rospy.Subscriber('/' + name + '/cart_pos', Pose, cart_pos)

        pub_grip = []

        if name == "ur5_1":
            pub_grip.append(rospy.Publisher("/" + name + "/gripper/command", Float64, queue_size=10 ))

        else:
            pub_grip.append(rospy.Publisher("/" + name + "/gripper_finger_1_joint_1/command", Float64, queue_size=10 ))
            pub_grip.append(rospy.Publisher("/" + name + "/gripper_finger_2_joint_1/command", Float64, queue_size=10 ))
            pub_grip.append(rospy.Publisher("/" + name + "/gripper_finger_middle_joint_1/command", Float64, queue_size=10 ))
            pub_grip.append(rospy.Publisher("/" + name + "/gripper_palm_finger_1_joint/command", Float64, queue_size=10 ))
        
        t = time.time()

        # Rate
        r = rospy.Rate(38)

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
                    if name == "ur5_1":
                        ex = (0.0 - act_pose_phantom.position.x)
                        ey = (0.0 - act_pose_phantom.position.y)
                        ez = (prev_grip_140 / scale_grip_140 - act_pose_phantom.position.z)

                    else:
                        if state_3f == 0:
                            ex = (0.0 - act_pose_phantom.position.x)
                            ey = (0.0 - act_pose_phantom.position.y)
                            ez = (prev_grip_3f_1 / scale_grip_3f - act_pose_phantom.position.z)

                        elif state_3f == 1:
                            ex = (0.0 - act_pose_phantom.position.x)
                            ey = (0.0 - act_pose_phantom.position.y)
                            ez = (prev_grip_3f_2 / scale_grip_3f - act_pose_phantom.position.z)
                            
                        elif state_3f == 2:
                            ex = (0.0 - act_pose_phantom.position.x)
                            ey = (0.0 - act_pose_phantom.position.y)
                            ez = (prev_grip_3f_mid / scale_grip_3f - act_pose_phantom.position.z)   

                        elif state_3f == 3:
                            ex = (0.0 - act_pose_phantom.position.x)
                            ey = (0.0 - act_pose_phantom.position.y)
                            ez = (prev_grip_3f_palm / scale_grip_3f_palm - act_pose_phantom.position.z)


                if ex < limit and ey < limit and ez < limit:
                    change = False        
                    print("###################")  

                        
            # 3 --   
            else:   
                if state == 0:
                    ex = (prev_y - pose.position.y)
                    ey = (prev_z - pose.position.z)
                    ez = (prev_x - pose.position.x)
                
                elif state == 1:
                    ex = (prev_pitch - pose.orientation.y)
                    ey = (prev_yaw - pose.orientation.z)
                    ez = (prev_roll - pose.orientation.x)

                elif state == 2:
                    ex = (0 - act_pose_phantom.position.x)
                    ey = (0 - act_pose_phantom.position.y)
                    ez0 = (0 - act_pose_phantom.position.z)

                    if name == "ur5_1":
                        ez = (prev_grip_140 - grip_140)      

                    else:
                        if state_3f == 0:
                            ez = (prev_grip_3f_1 - grip_3f_1)

                        elif state_3f == 1:
                            ez = (prev_grip_3f_2 - grip_3f_2)

                        elif state_3f == 2:
                            ez = (prev_grip_3f_mid - grip_3f_mid) 

                        elif state_3f == 3:
                            ez = (prev_grip_3f_palm - grip_3f_palm)

                else:
                    if name == "ur5_1":
                        pub_grip[0].publish(grip_140)

                    else:
                        if state_3f == 0:
                            pub_grip[0].publish(grip_3f_1)

                        elif state_3f == 0:
                            pub_grip[1].publish(grip_3f_2)

                        elif state_3f == 0:
                            pub_grip[2].publish(grip_3f_mid)

                        elif state_3f == 0:
                            pub_grip[3].publish(grip_3f_palm)
            

            wrench.wrench.force.x = ex * Ke - ex / (time.time() - t) * Kde
            wrench.wrench.force.y = ey * Ke - ey / (time.time() - t) * Kde
            wrench.wrench.force.z = ez * Ke - ez / (time.time() - t) * Kde

            if not change:
                wrench.wrench.force.y = wrench.wrench.force.y + 1.5


            if name == "ur5_2" and state == 2 and act_pose_phantom.position.z < 0:
                wrench.wrench.force.z = ez0 * Ke - ez0 / (time.time() - t) * Kde

            # 4 --
            pub_f.publish(wrench)  
        
            t = time.time()

            r.sleep()