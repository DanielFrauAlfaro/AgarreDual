#! /usr/bin/python3

from control_msgs.msg import JointControllerState
from geometry_msgs.msg import PoseStamped, Pose, WrenchStamped
from sensor_msgs.msg import Joy, CompressedImage
from std_msgs.msg import Int32
import rospy
from cv_bridge import CvBridge
import dearpygui.dearpygui as dpg
import numpy as np
import cv2


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

# Mensajes para las imagenes
frame_ = CompressedImage()
frame2_ = CompressedImage()
bridge = CvBridge()

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
vel_control = False     # Flag para el control en velocidad

change = True           # Flag para indicar cambio (a True para mandarlo al (0,0,0) al empezar)

first_frame_cam1 = False    # Flags para cuando llegue el primer frame de la cámara
first_frame_cam2 = False

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

# Ganancia del feedback de fuerza
K = 80
limit = 0.001


def shoulder_pan_listener(data):
    global q   
    q[0] = data.process_value
        
def shoulder_lift_listener(data):
    global q
    q[1] = data.process_value
    
def elbow_listener(data):
    global q
    q[2] = data.process_value
    
def wrist_1_listener(data):
    global q
    q[3] = data.process_value
    
def wrist_2_listener(data):
    global q
    q[4] = data.process_value
    
def wrist_3_listener(data):
    global q
    q[5] = data.process_value


# Callbacks de las camaras
def camera_cb(data):
    global frame_, first_frame_cam1, bridge
    frame_ = bridge.compressed_imgmsg_to_cv2(data)
    first_frame_cam1 = True
    
def camera_cb2(data):
    global bridge, frame2_, first_frame_cam2
    frame2_ = bridge.compressed_imgmsg_to_cv2(data)
    first_frame_cam2 = True

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
    - 2. Si no hay control en velocidad ...
        - 2.1 ... se almacena la POSICIÓN del robot a partir de las coordenadas del Phantom (reescaladas y con el offset del origen)
            También se guarda la POSICIÓN actual de la posición del Phantom
            La orientación del robot es la anterior registrada
            
        - 2.2 ... se almacena la ORIENTACIÓN del robot a partir de las coordenadas del Phantom (reescaladas y con el offset del origen)
            También se guarda la ORIENTACIÓN actual de la posición del Phantom
            La posición del robot es la anterior registrada
            
    - 3. Si el control es en velocidad ...
        - 3.1 ... Se incrementa la posición XYZ y la RPY se mantiene a 0 (no hay incremento)
        
        - 3.2 ... Se incrementa la orientación en RPY y la XYZ se mantiene a 0 (no hay incremento)
        
        - No se almacena la posición del Phantom, se tiene que ir a 0.0
'''
def cb(data):
    global pub, pose, scale_x, scale_y, scale_z1, scale_z2
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    global prev_pose_phantom, act_pose_phantom
    
    # 1. --
    act_pose_phantom.position = data.pose.position
    
    if not change:
        # 2 --
        if vel_control == False:
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
                    
            
        # 3 --
        else:
            if xyz:
                pose.position.x = data.pose.position.z
                pose.position.y = data.pose.position.x
                pose.position.z = data.pose.position.y  
                
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                
                # Se transforma la POSICIÓN del robot a la que debería tener en el Phantom
                prev_pose_phantom.position.x = (prev_y - or_y) / scale_y
                
                if prev_z > 0.0:
                    prev_pose_phantom.position.y = (prev_z - or_z) / scale_z1
                else:
                    prev_pose_phantom.position.y = (prev_z - or_z) / scale_z2
                
                prev_pose_phantom.position.z = (prev_x - or_x) / scale_x
                
                
            else:
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0

                pose.orientation.x = data.pose.position.z
                pose.orientation.y = data.pose.position.x 
                pose.orientation.z = data.pose.position.y

                # Se transforma la ORIENTACIÓN del robot a la que debería tener en el Phantom
                prev_pose_phantom.orientation.x = prev_y - or_pitch
                prev_pose_phantom.orientation.y = prev_z - or_yaw
                prev_pose_phantom.orientation.z = prev_x - or_roll
         
            
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

def cb_bt2(data):
    global vel_control, change, pubMoveType
    
    if data.buttons[0] == 1:
        change = True
        print("POS - VEL")
        if vel_control:
            vel_control = False
            pubMoveType.publish(0)  # Publica en el topic del robot para cambiar el modo de control 
        else:  
            vel_control = True
            pubMoveType.publish(1)  # Publica en el topic del robot para cambiar el modo de control
        

# Nodo
rospy.init_node("phantom_ctr")

# Publishers: pose al robot, fuerza al Phantom y modo de movimiento para el robot
pub = rospy.Publisher("/pose", Pose, queue_size=10)
pub_f = rospy.Publisher("/arm/servo_cf", WrenchStamped, queue_size=10)
pubMoveType = rospy.Publisher("/move_type", Int32, queue_size=10) 

# Subscribers: posición del phantom, posición del robot, botones y cámaras del robot
rospy.Subscriber("/arm/measured_cp", PoseStamped,cb)
rospy.Subscriber("/cart_pos", Pose, cart_cb)
rospy.Subscriber("/arm/button1", Joy, cb_bt1)
rospy.Subscriber("/arm/button2", Joy, cb_bt2)
rospy.Subscriber("/robot_camera/image_raw/compressed", CompressedImage, camera_cb)
rospy.Subscriber("/robot_camera2/image_raw/compressed", CompressedImage, camera_cb2)

rospy.Subscriber('/shoulder_pan_joint_position_controller/state', JointControllerState, shoulder_pan_listener)
rospy.Subscriber('/shoulder_lift_joint_position_controller/state', JointControllerState, shoulder_lift_listener)
rospy.Subscriber('/elbow_joint_position_controller/state', JointControllerState, elbow_listener)
rospy.Subscriber('/wrist_1_joint_position_controller/state', JointControllerState, wrist_1_listener)
rospy.Subscriber('/wrist_2_joint_position_controller/state', JointControllerState, wrist_2_listener)
rospy.Subscriber('/wrist_3_joint_position_controller/state', JointControllerState, wrist_3_listener)


# Espera a los primeros frames de las cámaras 
while first_frame_cam1 == False or first_frame_cam2 == False:
    pass


# Rate
r = rospy.Rate(20)

# Bucle infinito
'''
    - 1. Si se está en velocidad, el phantom intenta ir al centro para que si se suelta no mande incrementos.
       Si se está en posición, mantiene el Phantom sin moverse, compensando la gravedad (wrench obtenido experimentalmente)
    
    - 2. Si se pordujo un cambio (se presionó cualquiera de los botones para cambiar uno de los modos) ...
        - 2.1 ... Si hay control en POSICIÓN ...
            - 2.1.1 ... al cambio intentará volver a la anterior posición del Phantom registrada para la POSICIÓN del robot hasta cierto umbral
            - 2.1.2 ... al cambio intentará volver a la anterior posición del Phantom registrada para la ORIENTACIÓN del robot hasta cierto umbral
        
        - 2.2 ... Si hay control en VELOCIDAD mandará el Phantom al 0.0 (no considera posiciones anteriores, en velocidad siempre va al 0.0) 
        considerando cierto umbral
    
    - 3. Si no hubo cambio, se está funcionando normal, entonces envía las posiciones al robot
         
    - 4. Se publican los Wrenches calculados según el caso
    
    - 5. Se muestra la interfaz de usuario con las imágenes de las cámaras
'''
while not rospy.is_shutdown():
    # 1 --
    if not change:
        if vel_control:
            wrench.wrench.force.x = (0.0 - act_pose_phantom.position.x)*K
            wrench.wrench.force.y = (0.0 - act_pose_phantom.position.y)*K
            wrench.wrench.force.z = (0.0 - act_pose_phantom.position.z)*K
            
        
        else:
            wrench.wrench.force.x = 0.0
            wrench.wrench.force.y = 0.9
            wrench.wrench.force.z = 0.0
      
        
    # 2 --
    if change:
        print("change")
        # 2.1 --
        if not vel_control:
            # 2.1.1 --
            if xyz:
                wrench.wrench.force.x = (prev_pose_phantom.position.x - act_pose_phantom.position.x)*K
                wrench.wrench.force.y = (prev_pose_phantom.position.y - act_pose_phantom.position.y)*K
                wrench.wrench.force.z = (prev_pose_phantom.position.z - act_pose_phantom.position.z)*K
                
                if (prev_pose_phantom.position.x - act_pose_phantom.position.x) < limit and (prev_pose_phantom.position.y - act_pose_phantom.position.y) < limit and (prev_pose_phantom.position.z - act_pose_phantom.position.z) < limit:
                    change = False
                    print("########################################")
            
            # 2.1.2 --
            else:
                wrench.wrench.force.x = (prev_pose_phantom.orientation.x - act_pose_phantom.position.x)*K
                wrench.wrench.force.y = (prev_pose_phantom.orientation.y - act_pose_phantom.position.y)*K
                wrench.wrench.force.z = (prev_pose_phantom.orientation.z - act_pose_phantom.position.z)*K
                
                if (prev_pose_phantom.orientation.x - act_pose_phantom.position.x) < limit and (prev_pose_phantom.orientation.y - act_pose_phantom.position.y) < limit and (prev_pose_phantom.orientation.z - act_pose_phantom.position.z) < limit:
                    change = False
        
        # 2.2 --          
        else:
            wrench.wrench.force.x = (0.0 - act_pose_phantom.position.x)*K
            wrench.wrench.force.y = (0.0 - act_pose_phantom.position.y)*K
            wrench.wrench.force.z = (0.0 - act_pose_phantom.position.z)*K
            
            if (0.0 - act_pose_phantom.position.x) < limit and (0.0 - act_pose_phantom.position.y) < limit and (0.0 - act_pose_phantom.position.z) < limit:
                change = False
                  
    # 3 --   
    else:
        pub.publish(pose)
    
    # 4 --
    pub_f.publish(wrench)

    
    mode = ""
    
    if vel_control:
        mode = "Velocidad"
    else:
        mode = "Posicion"
    
    if xyz:
        mode = mode + "XYZ"
    else:
        mode = mode + "RPY"
    
    zeros = cv2.putText(zeros, "Modo: " + mode, (15,50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 0, 128), 2, cv2.LINE_AA)
    zeros = cv2.putText(zeros, "X: " + str(round(prev_x,2)) + "   Y: " + str(round(prev_y,2)) + "   Z: " + str(round(prev_z,2)), (15,100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 0), 1, cv2.LINE_AA)
    zeros = cv2.putText(zeros, "Roll: " + str(round(prev_roll,2)) + "  Pitch: " + str(round(prev_pitch,2)) + "  Yaw: " + str(round(prev_yaw,2)), (15,150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 0), 1, cv2.LINE_AA)
    
    zeros = cv2.putText(zeros, "Q0: " + str(round(q[0],2)), (15,200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 128), 1, cv2.LINE_AA)
    zeros = cv2.putText(zeros, "Q1: " + str(round(q[1],2)), (15,220), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 128), 1, cv2.LINE_AA)
    zeros = cv2.putText(zeros, "Q2: " + str(round(q[2],2)), (15,240), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 128), 1, cv2.LINE_AA)
    zeros = cv2.putText(zeros, "Q3: " + str(round(q[3],2)), (15,260), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 128), 1, cv2.LINE_AA)
    zeros = cv2.putText(zeros, "Q4: " + str(round(q[4],2)), (15,280), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 128), 1, cv2.LINE_AA)
    zeros = cv2.putText(zeros, "Q5: " + str(round(q[5],2)), (15,300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 128), 1, cv2.LINE_AA)

    
    numpy_horizontal = np.hstack((frame_, frame2_, zeros))
    cv2.imshow("UR5", numpy_horizontal)
    
    cv2.waitKey(1)
    r.sleep()
