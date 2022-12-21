#! /usr/bin/python3

from geometry_msgs.msg import PoseStamped, Pose, WrenchStamped
from sensor_msgs.msg import Joy, ImageCompressed
from std_msgs.msg import Int32
import rospy
from cv_bridge import CvBridge
import dearpygui.dearpygui as dpg
import numpy as np


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
frame_ = ImageCompressed()
frame2_ = ImageCompressed()
bridge = CvBridge()

# Factor de escala de los movimientos 
scale_x = 2
scale_y = 2
scale_z1 = 0.4  # Diferente según está por encima o por debajo de 0
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
K = 1
limit = 0.001

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
        
        if xyz:
            xyz = False
        else:  
            xyz = True

def cb_bt2(data):
    global vel_control, change, pubMoveType
    
    if data.buttons[0] == 1:
        change = True
        
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
rospy.Subscriber("/robot_camera/image_raw/compressed", ImageCompressed, camera_cb)
rospy.Subscriber("/robot_camera2/image_raw/compressed", ImageCompressed, camera_cb2)


# Espera a los primeros frames de las cámaras 
while first_frame_cam1 == False or first_frame_cam2 == False:
    pass


dpg.create_context()
dpg.create_viewport(title='Teleoperación', width=1460, height=1200)
dpg.setup_dearpygui()


data = np.flip(frame_, 2)  # because the camera data comes in as BGR and we need RGB
data = data.ravel()  # flatten camera data to a 1 d stricture
data = np.asfarray(data, dtype='f')  # change data type to 32bit floats
texture_data = np.true_divide(data, 255.0)  # normalize image data to prepare for GPU

data2 = np.flip(frame2_, 2)  # because the camera data comes in as BGR and we need RGB
data2 = data2.ravel()  # flatten camera data to a 1 d stricture
data2 = np.asfarray(data2, dtype='f')  # change data type to 32bit floats
texture_data2 = np.true_divide(data2, 255.0)  # normalize image data to prepare for GPU


with dpg.texture_registry(show=False):
    dpg.add_raw_texture(frame_.shape[1], frame_.shape[0], texture_data, tag="texture_tag", format=dpg.mvFormat_Float_rgb)
    dpg.add_raw_texture(frame2_.shape[1], frame2_.shape[0], texture_data2, tag="texture_tag2", format=dpg.mvFormat_Float_rgb)

    
with dpg.window(label="Cámara UR5", pos = [800,0]):
    dpg.add_image("texture_tag")
    
 
with dpg.window(label="Cámara UR52", pos = [800,500]):
    dpg.add_image("texture_tag2")
    

dpg.show_viewport()


# Rate
r = rospy.Rate(10)

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
    if vel_control:
        wrench.wrench.force.x = (0.0 - act_pose_phantom.position.x)*K
        wrench.wrench.force.y = (0.0 - act_pose_phantom.position.y)*K
        wrench.wrench.force.z = (0.0 - act_pose_phantom.position.z)*K
        
    
    else:
        wrench.wrench.force.x = 0.0
        wrench.wrench.force.y = 0.0
        wrench.wrench.force.z = 0.9
      
        
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
    
    
    # 5 --
    '''data = np.flip(frame_, 2)
    data = data.ravel()
    data = np.asfarray(data, dtype='f')
    texture_data = np.true_divide(data, 255.0)
    dpg.set_value("texture_tag", texture_data)
    
    data2 = np.flip(frame2_, 2)
    data2 = data2.ravel()
    data2 = np.asfarray(data2, dtype='f')
    texture_data2 = np.true_divide(data2, 255.0)
    dpg.set_value("texture_tag2", texture_data2)

    dpg.render_dearpygui_frame()'''
    
    
    r.sleep()
