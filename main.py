#! /usr/bin/python3

from __future__ import print_function
import dearpygui.dearpygui as dpg
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CompressedImage

from spatialmath import *
import roboticstoolbox as rtb

from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

# Origen en XYZ
or_x = 0.5095
or_y = 0.1334
or_z = 0.7347

# Origen RPY
or_roll = 1.57225399
or_pitch = 1.07079575
or_yaw = -0.001661

# Posiciones de XYZ y RPY
prev_x, prev_y, prev_z = or_x, or_y, or_z
prev_roll, prev_pitch, prev_yaw = or_roll , or_pitch, or_yaw

# Umbral para enviar datos
umbral = 0.01
first_frame_cam1 = False
first_frame_cam2 = False

# Flags para el modo de movimiento y para detectar si se presiona el checker
vel_controller = False
cb = False

# Mensajes de la camara
frame_ = CompressedImage()
frame2_ = CompressedImage()
bridge = CvBridge()


####### Callbacks #######
# Camaras: se pasan a formato numpy
def camera_cb(data):
    global frame_, first_frame_cam1 , bridge
    frame_ = bridge.compressed_imgmsg_to_cv2(data)
    
    first_frame_cam1 = True
    
def camera_cb2(data):
    global bridge, frame2_, first_frame_cam2
    frame2_ = bridge.compressed_imgmsg_to_cv2(data)
    
    first_frame_cam2 = True

# Callback para el topic de la posicion cartesiana del robot
def cart_cb(data):
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    
    prev_x = data.position.x
    prev_y = data.position.y
    prev_z = data.position.z
    
    prev_roll = data.orientation.x
    prev_pitch = data.orientation.y
    prev_yaw = data.orientation.z

# Callback del checker: publica modo seleccionado
def callbackCheck(sender, app_data, user_data):
    global vel_controller, cb
    cb = True
    vel_controller = dpg.get_value(sender)
    if vel_controller == True:
        pubMoveType.publish(1)
    else:
        pubMoveType.publish(0)         

### Slider ###
# XYZ posicion: se mantiene los RPY y se cambian los XYZ al pasar un umbral
def callbackSlider(sender, app_data, user_data):
    global prev_x, prev_roll
    global prev_y, prev_pitch
    global prev_z, prev_yaw
    global pub
    
    data = dpg.get_value(sender)
    x = data[0]
    y = data[1]
    z = data[2]
    
    if abs(x-prev_x) > umbral or abs(y-prev_y) > umbral or abs(z-prev_z) > umbral:
        
        pose = Pose()
        
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        pose.orientation.x = prev_roll
        pose.orientation.y = prev_pitch
        pose.orientation.z = prev_yaw
        pose.orientation.w = 1
        
        pub.publish(pose)

# RPY posicion: se mantiene los XYZ y se cambian los RPY al pasar un umbral
def callbackSlider2(sender, app_data, user_data):
    global prev_x, prev_roll
    global prev_y, prev_pitch
    global prev_z, prev_yaw
    global pub
    
    data = dpg.get_value(sender)
    x = data[0]
    y = data[1]
    z = data[2]
    if abs(x-prev_x) > umbral or abs(y-prev_y) > umbral or abs(z-prev_z) > umbral:
        pose = Pose()
            
        pose.position.x = prev_x
        pose.position.y = prev_y
        pose.position.z = prev_z

        pose.orientation.x = x
        pose.orientation.y = y
        pose.orientation.z = z
        pose.orientation.w = 1
            
        pub.publish(pose)

# XYZ velocidad: se hacen por incrementos de XYZ, por lo que los RPY estan a 0.0
def callbackSlider_(sender, app_data, user_data):
    global prev_x, prev_roll
    global prev_y, prev_pitch
    global prev_z, prev_yaw
    global pub
    
    data = dpg.get_value(sender)
    x = data[0]
    y = data[1]
    z = data[2]
    
    pose = Pose()
       
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0
       
    pub.publish(pose)

# RPY velocidad: se hacen incrementos de RPY, por lo que los XYZ estan a 0.0
def callbackSlider2_(sender, app_data, user_data):
    global prev_x, prev_roll
    global prev_y, prev_pitch
    global prev_z, prev_yaw
    global pub
    
    data = dpg.get_value(sender)
    x = data[0]
    y = data[1]
    z = data[2]
    
    pose = Pose()
            
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0

    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = 1
            
    pub.publish(pose)


# Variables de ROS:
# Nodo
rospy.init_node("nodo")

# Publosher de la pose y el modo de movimiento
pub = rospy.Publisher("/pose", Pose, queue_size=10)
pubMoveType = rospy.Publisher("/move_type", Int32, queue_size=10) 

# Subscribers de la pose del robot y las dos cámaras
rospy.Subscriber("/cart_pos", Pose, cart_cb)
rospy.Subscriber("/robot_camera/image_raw/compressed", CompressedImage, camera_cb)
rospy.Subscriber("/robot_camera2/image_raw/compressed", CompressedImage, camera_cb2)


# Se espera a tener los dos frames de las camaras (la simulacion esta funcionandos)
while first_frame_cam1 ==False or first_frame_cam2 == False:
    pass

# GUI
dpg.create_context()
dpg.create_viewport(title='Teleoperación', width=1460, height=1200)
dpg.setup_dearpygui()

# Se crean los sliders en las posiciones iniciales
with dpg.window(tag="Controlador", width=800, height=640):
    dpg.add_3d_slider(label="Position XYZ", tag="position_slider", default_value=[0.5095, 0.1334, 0.7347], min_x=-0.7, max_x=0.7, min_y=-0.55, max_y=0.55, min_z=0.0, max_z=0.85, height=200, width=200,callback=callbackSlider, user_data="slider", )
    dpg.add_3d_slider(label="Position RPY", tag="rotation_slider", default_value=[1.57225399 , 1.07079575, -0.001661], min_x=-3.14, max_x=3.14, min_y=-3.14, max_y=3.14, min_z=-3.14, max_z=3.14, height=200, width=200,callback=callbackSlider2, user_data="slider")

    dpg.add_3d_slider(label="Velocity XYZ", show=False, tag="position_slider_v", default_value=[0.0, 0.0, 0.0], min_x=-0.1, max_x=0.1, min_y=-0.1, max_y=0.1, min_z=-0.1, max_z=0.1, height=200, width=200,callback=callbackSlider_, user_data="slider")
    dpg.add_3d_slider(label="Velocity RPY", show = False, tag="rotation_slider_v", default_value=[0.0, 0.0, 0.0], min_x=-0.1, max_x=0.1, min_y=-0.1, max_y=0.1, min_z=-0.1, max_z=0.1, height=200, width=200,callback=callbackSlider2_, user_data="slider")


# Se crea el checker para el cambio de modo
with dpg.window(tag="Ckeck", pos=[0,640]):
    dpg.add_checkbox(label="Activar control en velocidad", tag="vel_control",
                            callback=callbackCheck, user_data="vel_control")


# Se procesan los primeros frames de las cámaras para mostrarlos por la GUI
data = np.flip(frame_, 2)  
data = data.ravel()  
data = np.asfarray(data, dtype='f') 
texture_data = np.true_divide(data, 255.0)  

data2 = np.flip(frame2_, 2)  
data2 = data2.ravel() 
data2 = np.asfarray(data2, dtype='f')  
texture_data2 = np.true_divide(data2, 255.0)  

# Mostrar imagenes para la GUI
with dpg.texture_registry(show=False):
    dpg.add_raw_texture(frame_.shape[1], frame_.shape[0], texture_data, tag="texture_tag", format=dpg.mvFormat_Float_rgb)
    dpg.add_raw_texture(frame2_.shape[1], frame2_.shape[0], texture_data2, tag="texture_tag2", format=dpg.mvFormat_Float_rgb)

    
with dpg.window(label="Cámara UR5", pos = [800,0]):
    dpg.add_image("texture_tag")
    
 
with dpg.window(label="Cámara UR52", pos = [800,500]):
    dpg.add_image("texture_tag2")

# Rate
r = rospy.Rate(40)

# Muestra las ventanas
dpg.show_viewport()

# Bucle infinito
while dpg.is_dearpygui_running():

    # Se procesan las imagenes
    data = np.flip(frame_, 2)
    data = data.ravel()
    data = np.asfarray(data, dtype='f')
    texture_data = np.true_divide(data, 255.0)
    dpg.set_value("texture_tag", texture_data)
    
    data2 = np.flip(frame2_, 2)
    data2 = data2.ravel()
    data2 = np.asfarray(data2, dtype='f')
    texture_data2 = np.true_divide(data2, 255.0)
    dpg.set_value("texture_tag2", texture_data2)

    # Renderizado
    dpg.render_dearpygui_frame()
    
    # Si hubo un cambio de modo, muestra otros sliders y cambia sus valores
    if cb:
        if vel_controller == True:
            dpg.configure_item("position_slider", show=False)
            dpg.configure_item("rotation_slider", show=False)
            dpg.configure_item("position_slider_v", show=True, label="Velocity XYZ", default_value=[0, 0, 0])
            dpg.configure_item("rotation_slider_v", show=True, label="Velocity RPY", default_value=[0, 0, 0])
        else:
            dpg.configure_item("position_slider_v", show=False)
            dpg.configure_item("rotation_slider_v", show=False)
            dpg.configure_item("position_slider", show=True, label="Position XYZ", default_value=[prev_x, prev_y, prev_z])
            dpg.configure_item("rotation_slider", show=True, label="Position RPY", default_value=[prev_roll, prev_pitch, prev_yaw])
        
        cb = False

    r.sleep()
    
# On ctrl+c client.stop()# pybullet