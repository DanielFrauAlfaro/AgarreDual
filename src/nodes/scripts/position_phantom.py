#! /usr/bin/python3

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Pose
import nodes.msg
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
cmp_poses = nodes.msg.Poses()

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
act_pose_phantom = Pose()

act_pose_phantom.position.x = -1.0
act_pose_phantom.position.y = -1.0
act_pose_phantom.position.z = -1.0
act_pose_phantom.orientation.x = -1.0
act_pose_phantom.orientation.y = -1.0
act_pose_phantom.orientation.z = -1.0

# Tiempo para coger los mensajes del /joint_states
interval = 0.0
prev = time.time()

# Nombre del robot
name = ""



# TODO: comprobar que esto funciona, que publicando en el callback va mejor que un bucle de control
# Publishers: pose al robot, fuerza al Phantom y modo de movimiento para el robot
pub = rospy.Publisher("/" + name + "/pose", Pose, queue_size=10)
pub_cmp_pose = rospy.Publisher("/" + name + "/cmp_pose", nodes.msg.Poses, queue_size=10)



# Callback de las posiciones del Phantom
'''
Las coordenadas del Phantom son diferentes de las del robot --> 
  - x_robot = z_phantom
  - y_robot = x_phantom
  - z_robot = y_phantom
  
Se actúa sobre una variable: 
  - pose: comando de posición al robot
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
    global pub, pub_cmp_pose
    global pose, cmp_poses
    global scale_x, scale_y, scale_z1, scale_z2
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    global act_pose_phantom
    global grip_140, grip_3f_1, grip_3f_2, grip_3f_mid, grip_3f_palm
    global scale_grip_140, scale_grip_3f
    global state_3f
    
    
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

            pub.publish(pose)
            
        # 2.2 --    
        elif state == 1:
            pose.position.x = prev_x
            pose.position.y = prev_y
            pose.position.z = prev_z

            pose.orientation.x = data.pose.position.z + or_roll
            pose.orientation.y = data.pose.position.x + or_pitch
            pose.orientation.z = data.pose.position.y + or_yaw

            pub.publish(pose)

        # 2.3 --
        elif state == 2:
            if name == "ur5_1":
                grip_140 = data.pose.position.z * scale_grip_140
                cmp_poses.g140 = grip_140
            
            else:
                if state_3f == 0:
                    grip_3f_1 = data.pose.position.z * scale_grip_3f
                    cmp_poses.g3f_1 = grip_3f_1

                elif state_3f == 1:
                    grip_3f_2 = data.pose.position.z * scale_grip_3f
                    cmp_poses.g3f_2 = grip_3f_2

                elif state_3f == 2:
                    grip_3f_mid = data.pose.position.z * scale_grip_3f
                    cmp_poses.g3f_m = grip_3f_mid

                elif state_3f == 3:
                    grip_3f_palm = data.pose.position.z * scale_grip_3f_palm
                    cmp_poses.g3f_p = grip_3f_palm

        # TODO: implementar publicador para la pinza

        cmp_poses.poses[0] = act_pose_phantom
        cmp_poses.poses[1] = pose

        pub_cmp_pose.publish(cmp_poses)
        



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



if __name__ == "__main__":
    print(sys.argv)


    if len(sys.argv) > 0:
        
        # name = sys.argv[1]
        name = "ur5_2"
        name_p = "arm"
        
        # Nodo
        rospy.init_node(name + "_phantom_pos_ctr")

        # Subscribers: posición del phantom, posición del robot, botones y cámaras del robot
        rospy.Subscriber("/" + name_p + "/measured_cp", PoseStamped,cb)
        rospy.Subscriber("/" + name_p + "/button1", Joy, cb_bt1)
        rospy.Subscriber("/" + name_p + "/button2", Joy, cb_bt2)

        pub_grip = []

        if name == "ur5_1":
            pub_grip.append(rospy.Publisher("/" + name + "/gripper/command", Float64, queue_size=10 ))

        else:
            pub_grip.append(rospy.Publisher("/" + name + "/gripper_finger_1_joint_1/command", Float64, queue_size=10 ))
            pub_grip.append(rospy.Publisher("/" + name + "/gripper_finger_2_joint_1/command", Float64, queue_size=10 ))
            pub_grip.append(rospy.Publisher("/" + name + "/gripper_finger_middle_joint_1/command", Float64, queue_size=10 ))
            pub_grip.append(rospy.Publisher("/" + name + "/gripper_palm_finger_1_joint/command", Float64, queue_size=10 ))

        # Rate
        r = rospy.Rate(38)
        
        rospy.spin()