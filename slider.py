#! /usr/bin/python3

from __future__ import print_function
import dearpygui.dearpygui as dpg
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CompressedImage

from spatialmath import SE3
import roboticstoolbox as rtb

from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import time

# Origen en XYZ
or_x = 0.5137                        # Origen del movimiento cartesiano
or_y = 0.1334                       
or_z = 0.4397

or_roll = -2.296 # 0.832 # 0.87                # Origen del movimiento articular de la muñeca del robot
or_pitch = 0.0 # 1.12                   # YXZ hacia abajo # YXZ # XYZ
or_yaw = -3.03


# Posiciones de XYZ y RPY
prev_x, prev_y, prev_z = or_x, or_y, or_z
prev_roll, prev_pitch, prev_yaw = or_roll , or_pitch, or_yaw

# Umbral para enviar datos
umbral = 0.0
first_frame_cam1 = False
first_frame_cam2 = False

# Flags para el modo de movimiento y para detectar si se presiona el checker
vel_controller = False
cb = False

# Mensajes de la camara
frame_ = CompressedImage()
frame2_ = CompressedImage()
bridge = CvBridge()

pose = Pose()

interval = 0.11
prev = time.time()

interval2 = 0.11
prev2 = time.time()

### Slider ###
# XYZ posicion: se mantiene los RPY y se cambian los XYZ al pasar un umbral
def callbackSlider(sender, app_data, user_data):
    global prev_x, prev_roll
    global prev_y, prev_pitch
    global prev_z, prev_yaw
    global pub, interval, prev, pose
    
    if time.time() - prev > interval:         # Solo se ejecuta cuando pasa el intervalo
            
        prev = time.time() 

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

            prev_x = x
            prev_y = y
            prev_z = z
            
            pub.publish(pose)

# RPY posicion: se mantiene los XYZ y se cambian los RPY al pasar un umbral
def callbackSlider2(sender, app_data, user_data):
    global prev_x, prev_roll
    global prev_y, prev_pitch
    global prev_z, prev_yaw
    global pub, prev2, interval2
    
    
    if time.time() - prev2 > interval2:         # Solo se ejecuta cuando pasa el intervalo
            
        prev2 = time.time() 
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

            prev_roll = x
            prev_pitch = y
            prev_yaw = z
                
            pub.publish(pose)

mode = False

def cb_check(sender, app_data, user_data):
    global mode

    b = Bool()

    if not mode:
        dpg.configure_item("position_slider", show=False)
        dpg.configure_item("rotation_slider", show=True)
        mode = True
    else:
        dpg.configure_item("position_slider", show=True)
        dpg.configure_item("rotation_slider", show=False)
        mode = False
        

name = "ur5_2"
rospy.init_node("nodo_"+name)

# Publosher de la pose y el modo de movimiento
pub = rospy.Publisher("/"+name+"/pose", Pose, queue_size=10)

# GUI
dpg.create_context()
dpg.create_viewport(title='Teleoperación', width=700, height=600)
dpg.setup_dearpygui()

# Se crean los sliders en las posiciones iniciales
with dpg.window(tag="Controlador", width=600, height=600):
    dpg.add_3d_slider(label="Position XYZ", tag="position_slider", default_value=[0.5095, 0.1334, 0.7347], min_x=-1, max_x=0.7, min_y=-0.55, max_y=0.55, min_z=0.0, max_z=0.85, height=200, width=200,callback=callbackSlider, user_data="slider", )
    dpg.add_3d_slider(show=False, label="Position RPY", tag="rotation_slider", default_value=[or_roll, or_pitch, or_yaw], min_x=-3.14, max_x=3.14, min_y=-3.14, max_y=3.14, min_z=-3.14, max_z=3.14, height=200, width=200,callback=callbackSlider2, user_data="slider")

    dpg.add_checkbox(callback=cb_check)


# Rate
r = rospy.Rate(20)

# Muestra las ventanas
dpg.show_viewport()
dpg.start_dearpygui()