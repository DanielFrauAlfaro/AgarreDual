#! /usr/bin/python3

from __future__ import print_function
import dearpygui.dearpygui as dpg
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

from spatialmath import *
import roboticstoolbox as rtb

from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

or_x = 0.5095
or_y = 0.1334
or_z = 0.7347

or_roll = 1.57225399
or_pitch = 1.07079575
or_yaw = -0.001661

prev_x, prev_y, prev_z = or_x, or_y, or_z
prev_roll, prev_pitch, prev_yaw = or_roll , or_pitch, or_yaw

umbral = 0.01
ready = False
vel_controller = False
cb = False
first_frame_cam1 = False
first_frame_cam2 = False

frame_ = Image()
frame2_ = Image()
bridge = CvBridge()



def camera_cb(data):
    global frame_, first_frame_cam1 , bridge
    frame_ = bridge.imgmsg_to_cv2(data)
    first_frame_cam1 = True
    
def camera_cb2(data):
    global bridge, frame2_, first_frame_cam2
    frame2_ = bridge.imgmsg_to_cv2(data)
    
    first_frame_cam2 = True
    
def cart_cb(data):
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    
    prev_x = data.position.x
    prev_y = data.position.y
    prev_z = data.position.z
    
    prev_roll = data.orientation.x
    prev_pitch = data.orientation.y
    prev_yaw = data.orientation.z

def callbackCheck(sender, app_data, user_data):
    global vel_controller, cb
    cb = True
    vel_controller = dpg.get_value(sender)
    if vel_controller == True:
        pubMoveType.publish(1)
    else:
        pubMoveType.publish(0)         
   
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



rospy.init_node("nodo")

pub = rospy.Publisher("/pose", Pose, queue_size=10)
pubMoveType = rospy.Publisher("/move_type", Int32, queue_size=10) 

rospy.Subscriber("/cart_pos", Pose, cart_cb)
rospy.Subscriber("/robot_camera/image_raw", Image, camera_cb)
rospy.Subscriber("/robot_camera2/image_raw", Image, camera_cb2)


while first_frame_cam1 ==False or first_frame_cam2 == False:
    pass


dpg.create_context()
dpg.create_viewport(title='Teleoperación', width=1460, height=1200)
dpg.setup_dearpygui()

with dpg.window(tag="Controlador", width=800, height=640):
    dpg.add_3d_slider(label="Position XYZ", tag="position_slider", default_value=[0.5095, 0.1334, 0.7347], min_x=-0.7, max_x=0.7, min_y=-0.55, max_y=0.55, min_z=0.0, max_z=0.85, height=200, width=200,callback=callbackSlider, user_data="slider", )
    dpg.add_3d_slider(label="Position RPY", tag="rotation_slider", default_value=[1.57225399 , 1.07079575, -0.001661], min_x=-3.14, max_x=3.14, min_y=-3.14, max_y=3.14, min_z=-3.14, max_z=3.14, height=200, width=200,callback=callbackSlider2, user_data="slider")

    dpg.add_3d_slider(label="Velocity XYZ", show=False, tag="position_slider_v", default_value=[0.0, 0.0, 0.0], min_x=-0.1, max_x=0.1, min_y=-0.1, max_y=0.1, min_z=-0.1, max_z=0.1, height=200, width=200,callback=callbackSlider_, user_data="slider")
    dpg.add_3d_slider(label="Velocity RPY", show = False, tag="rotation_slider_v", default_value=[0.0, 0.0, 0.0], min_x=-0.1, max_x=0.1, min_y=-0.1, max_y=0.1, min_z=-0.1, max_z=0.1, height=200, width=200,callback=callbackSlider2_, user_data="slider")


with dpg.window(tag="Ckeck", pos=[0,640]):
    dpg.add_checkbox(label="Activar control en velocidad", tag="vel_control",
                            callback=callbackCheck, user_data="vel_control")

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
while dpg.is_dearpygui_running():
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

    dpg.render_dearpygui_frame()
    
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



# On ctrl+c client.stop()# pybullet