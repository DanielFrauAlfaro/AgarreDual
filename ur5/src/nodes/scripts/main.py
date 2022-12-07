#! /usr/bin/python3

from __future__ import print_function
import dearpygui.dearpygui as dpg
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2 as cv
import numpy as np


rospy.init_node("nodo")
pub = rospy.Publisher("/pose", Pose, queue_size=10)

dpg.create_context()

max_limit = 0.2
min_limit = -0.2
umbral = 0.01
ready = False

prev_x, prev_y, prev_z= 0.4919, 0.1092, 0.6578
def callbackSlider(sender, app_data, user_data):
    global prev_x
    global prev_y
    global prev_z
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

        pose.orientation.x = 0.360399
        pose.orientation.y = 0.3611085
        pose.orientation.z = 0.607862
        pose.orientation.w = 0.60845
        
        pub.publish(pose)
        
        prev_x = x
        prev_y = y
        prev_z = z

frame_ = Image()
bridge = CvBridge()
a = False
def camera_cb(data):
    global frame_, a, bridge
    frame_ = bridge.imgmsg_to_cv2(data)
    a = True

rospy.Subscriber("/robot_camera/image_raw", Image, camera_cb)

while a==False:
    b = 1

dpg.create_context()
dpg.create_viewport(title='Teleoperación', width=1460, height=800)
dpg.setup_dearpygui()

with dpg.window(tag="Controlador", width=800, height=800):
    dpg.add_3d_slider(label="3D Slider", default_value=[0.5095, 0.1334, 0.7347], min_x=-0.6, max_x=0.6, min_y=-0.55, max_y=0.55, min_z=0.1, max_z=0.75, height=200, width=200,callback=callbackSlider, user_data="slider")


data = np.flip(frame_, 2)  # because the camera data comes in as BGR and we need RGB
data = data.ravel()  # flatten camera data to a 1 d stricture
data = np.asfarray(data, dtype='f')  # change data type to 32bit floats
texture_data = np.true_divide(data, 255.0)  # normalize image data to prepare for GPU


with dpg.texture_registry(show=False):
    dpg.add_raw_texture(frame_.shape[1], frame_.shape[0], texture_data, tag="texture_tag", format=dpg.mvFormat_Float_rgb)

with dpg.window(label="Cámara UR5", pos = [800,0]):
    dpg.add_image("texture_tag")

dpg.show_viewport()
while dpg.is_dearpygui_running():
    data = np.flip(frame_, 2)
    data = data.ravel()
    data = np.asfarray(data, dtype='f')
    texture_data = np.true_divide(data, 255.0)
    dpg.set_value("texture_tag", texture_data)

    dpg.render_dearpygui_frame()


# On ctrl+c client.stop()# pybullet