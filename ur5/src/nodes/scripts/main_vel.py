#! /usr/bin/python3

from __future__ import print_function
import dearpygui.dearpygui as dpg
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
import rospy
from geometry_msgs.msg import Pose

rospy.init_node("nodo")
pub = rospy.Publisher("/pose", Pose, queue_size=10)

dpg.create_context()

max_limit = 0.2
min_limit = -0.2
umbral = 0.01
ready = False

# Connect to ros master

prev_x, prev_y, prev_z= 0.4919, 0.05092, 0.6578
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

        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        
        pub.publish(pose)
        
        prev_x = x
        prev_y = y
        prev_z = z
        



with dpg.window(tag="Primary Window"):
    dpg.add_3d_slider(label="3D Slider", default_value=[0.0, 0.0, 0.0], min_x=-0.05, max_x=0.05, min_y=-0.05, max_y=0.05, min_z=-0.05, max_z=0.05, height=200, width=200, callback=callbackSlider, user_data="slider")
    """ dpg.add_button(label="Save")
    dpg.add_input_text(label="string", default_value="Quick brown fox")
    dpg.add_slider_float(label="float", default_value=0.273, max_value=1) """
    

dpg.create_viewport(title='Interfaz', width=600, height=600)
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.set_primary_window("Primary Window", True)
dpg.start_dearpygui()
dpg.destroy_context()

# On ctrl+c client.stop()# pybullet