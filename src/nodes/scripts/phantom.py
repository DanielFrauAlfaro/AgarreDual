#! /usr/bin/python3

from std_msgs.msg import Float64, Int32, Float32MultiArray
from geometry_msgs.msg import PoseStamped, WrenchStamped, Pose
from sensor_msgs.msg import Joy
import rospy
import sys
import time

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
st = Int32()

grip_140 = Float64()

grip_1 = Float64()
grip_2 = Float64()
grip_mid = Float64()
grip_palm = Float64()

# Factor de escala de los movimientos 
scale_x = 2
scale_y = 2
scale_z1 = 3.5 # Diferente según está por encima o por debajo de 0
scale_z2 = 7

scale_grip_140 = 4.08
scale_grip = 5
scale_grip_palm = 0.5

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
prev_grip_1, prev_grip_2, prev_grip_mid, prev_grip_palm = 0, 0, 0, 0

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
name = "ur5_2"
grip = "3f"

# Publisher del estado de la máquina
pub_state = rospy.Publisher("/" + name + "/state", Int32, queue_size=10)


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
    global grip_140, grip_1, grip_2, grip_mid, grip_palm
    global scale_grip_140, scale_grip
    global state_3f
    global grip
    
    
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
                if grip == "2f":
                    grip_140 = data.pose.position.z * scale_grip_140
                
                else:
                    if state_3f == 0:
                        grip_1 = data.pose.position.x * scale_grip
                        grip_2 = data.pose.position.y * scale_grip
                        grip_mid = data.pose.position.z * scale_grip

                    elif state_3f == 1:
                        grip_palm = data.pose.position.z * scale_grip_palm


# Callbacks de los botones
def cb_bt1(data):
    global change, state, pub_state, st

    if data.buttons[0] == 1:
        state = state + 1       # Incrementa el estado
        change = True 

        if state > 2:           # Si se pasa del número de estados, ...
            state = 0           # ... vuelve al estado 0 ...
    
    st = state
    
    pub_state.publish(st)
        
def cb_bt2(data):
    global change
    global state, state_3f
    
    if state == 2:
        if data.buttons[0] == 1:
            state_3f = state_3f + 1
            change = True

            if state_3f > 1:
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

def grip_pos(data):
    global grip
    global prev_grip_1, prev_grip_2, prev_grip_mid, prev_grip_palm
    global prev_grip_140

    if grip == "2f":
        prev_grip_140 = data.data[0]

    elif grip == "3f":
        prev_grip_1 = data.data[0]
        prev_grip_2 = data.data[1]
        prev_grip_mid = data.data[2]
        prev_grip_palm = data.data[3]



if __name__ == "__main__":

    if len(sys.argv) > 0:
        
        name = sys.argv[1]

        if sys.argv[2] == '3f':
            print("3f")
            grip = "3f"
        elif sys.argv[2] == '2f_140':
            print("2f")
            grip = "2f"
        else:
            grip = ""
        

        # Nodo
        rospy.init_node(name + "_phantom_ctr")

        pub_state = rospy.Publisher("/" + name + "/state", Int32, queue_size=10)

        # Publishers: pose al robot, fuerza al Phantom y modo de movimiento para el robot
        pub = rospy.Publisher("/" + name + "/pose", Pose, queue_size=10)
        pub_f = rospy.Publisher("/" + name + "/servo_cf", WrenchStamped, queue_size=10)

        # Subscribers: posición del phantom, posición del robot, botones y cámaras del robot
        rospy.Subscriber("/" + name + "/measured_cp", PoseStamped,cb)
        rospy.Subscriber("/" + name + "/button1", Joy, cb_bt1)
        rospy.Subscriber("/" + name + "/button2", Joy, cb_bt2)
        rospy.Subscriber('/' + name + '/cart_pos', Pose, cart_pos)
        rospy.Subscriber("/" + name + "/grip_pos", Float32MultiArray, grip_pos)

        pub_grip = []

        if grip != "":
            if grip == "2f":
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
                    if grip == "2f":
                        ex = (0.0 - act_pose_phantom.position.x)
                        ey = (0.0 - act_pose_phantom.position.y)
                        ez = (prev_grip_140 / scale_grip_140 - act_pose_phantom.position.z)

                    else:
                        if state_3f == 0:
                            ex = (prev_grip_1 - act_pose_phantom.position.x)
                            ey = (prev_grip_2 - act_pose_phantom.position.y)
                            ez = (prev_grip_mid / scale_grip - act_pose_phantom.position.z)

                        elif state_3f == 1:
                            ex = (0.0 - act_pose_phantom.position.x)
                            ey = (0.0 - act_pose_phantom.position.y)
                            ez = (prev_grip_palm / scale_grip_palm - act_pose_phantom.position.z)


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

                    if grip == "2f":
                        ez = (prev_grip_140 - grip_140)      

                    else:
                        if state_3f == 0:
                            ex = (prev_grip_1 - grip_1)
                            ey = (prev_grip_2 - grip_2)
                            ez = (prev_grip_mid - grip_mid) 

                        elif state_3f == 3:
                            ez = (prev_grip_palm - grip_palm)


# --------------------------------------------------------------------------------------------------


                if state != 2:
                    pub.publish(pose)

                else:
                    if name == "ur5_1":
                        pub_grip[0].publish(grip_140)

                    else:
                        if state_3f == 0:
                            pub_grip[0].publish(grip_1)
                            pub_grip[1].publish(grip_2)
                            pub_grip[2].publish(grip_mid)

                        elif state_3f == 1:
                            pub_grip[3].publish(grip_palm)
            

            wrench.wrench.force.x = ex * Ke - ex / (time.time() - t) * Kde
            wrench.wrench.force.y = ey * Ke - ey / (time.time() - t) * Kde
            wrench.wrench.force.z = ez * Ke - ez / (time.time() - t) * Kde

            if not change:
                wrench.wrench.force.y = wrench.wrench.force.y + 1.5


            if state == 2 and act_pose_phantom.position.z < 0:
                wrench.wrench.force.z = ez0 * Ke - ez0 / (time.time() - t) * Kde

            # 4 --
            pub_f.publish(wrench)  
        
            t = time.time()

            r.sleep()