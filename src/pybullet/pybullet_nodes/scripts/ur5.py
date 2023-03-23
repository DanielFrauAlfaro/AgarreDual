#!/usr/bin/env python3

import pybullet as p
import sys
import rospy
from ur5_class import UR5e
from plane import Plane
from std_msgs.msg import String
import numpy as np
import math
import matplotlib.pyplot as plt

# Client
client = 0

rendered_img = plt.imshow(np.zeros((100, 100, 3)))
fps = 5000000

cam_roll, cam_pitch, cam_yaw = 0.0, 0.0, 3.14
cam_pos = [1, 0.0, 0.4]

# Computes projection matrix
proj_matrix = p.computeProjectionMatrixFOV(fov = 80, 
                                            aspect = 1, 
                                            nearVal = 0.01, 
                                            farVal = 100,
                                            physicsClientId = client)

# Rotation matrices
rot_x = np.array([[1, 0, 0], 
                    [0, math.cos(cam_roll), -math.sin(cam_roll)], 
                    [0, math.sin(cam_roll), math.cos(cam_roll)]])

rot_y = np.array([[math.cos(cam_pitch),0, math.sin(cam_pitch)], 
                    [0, 1, 0], 
                    [-math.sin(cam_pitch),0,math.cos(cam_pitch)]])

rot_z = np.array([[math.cos(cam_yaw), -math.sin(cam_yaw), 0], 
                    [math.sin(cam_yaw), math.cos(cam_yaw), 0], 
                    [0, 0, 1]])

# Position and orientation
pos = cam_pos
rot_mat = np.matmul(np.matmul(rot_x, rot_y), rot_z)

# Calculates the camera vector and the up vector
camera_vec = np.matmul(rot_mat, [1, 0, 0])
up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))

# Computes the view matrix
view_matrix = p.computeViewMatrix(cameraEyePosition = pos, 
                                    cameraTargetPosition = pos + camera_vec,
                                    cameraUpVector = up_vec,
                                    physicsClientId = client)


# Callback to spawn the desired object
def spawn_cb(data):
    global client
    
    s = data.data.split()
    name = s[0]
    pos = [float(s[1]), float(s[2]), float(s[3])]
    orient = p.getQuaternionFromEuler(eulerAngles=[float(s[4]), float(s[5]), float(s[6])],
                                      physicsClientId=client)

    p.loadURDF(fileName=s[-1] + "/src/objects_models/urdf/" + name + "/model.urdf",
                basePosition=pos,
                baseOrientation=orient)


# Render function
def render():
    global client
    global cam_roll, cam_pitch, cam_yaw
    global cam_pos
    global rendered_img
    global view_matrix, proj_matrix

    # Shows the image
    frame = p.getCameraImage(width = 100, 
                                height = 100, 
                                viewMatrix = view_matrix, 
                                projectionMatrix = proj_matrix, 
                                physicsClientId = client)[2]
    # frame = np.reshape(frame, (100, 100, 4))
    rendered_img.set_data(frame)
    plt.draw()
    plt.pause(1/fps) 


# Main
if __name__ == "__main__":

    # Node 
    rospy.init_node("master_pybullet")
    
    # Starts the simulation and applies gravity
    client = p.connect(p.GUI)
    p.setGravity(0, 0, -10, physicsClientId=client) 
    
    # Removes GUI elements from the simulation
    p.configureDebugVisualizer(flag = p.COV_ENABLE_GUI, 
                               enable=0)

    # Approaches camera point of view
    width, height, viewMatrix, projectionMatrix, cameraUp, cameraForward, horizontal, vertical, yaw, pitch, dist, target = p.getDebugVisualizerCamera(physicsClientId = client)
    p.resetDebugVisualizerCamera(cameraDistance=dist-3.3,
                                cameraYaw=yaw, 
                                cameraPitch=pitch,
                                cameraTargetPosition=target,
                                physicsClientId=client)


    # ------ Decodifies the arguments ------
    # Number of robots
    n = int(sys.argv[1])

    # Robot names
    names = [sys.argv[2], sys.argv[3]]
    
    # Origin of each reference system
    origin = [[float(sys.argv[5]), float(sys.argv[7]), float(sys.argv[9])], 
              [float(sys.argv[11]), float(sys.argv[13]), float(sys.argv[15])]]
    
    # Gripper naes
    grip = [sys.argv[16], sys.argv[17]]

    # Source directory
    dir = sys.argv[18]

    # List of UR5e objects
    ur = []

    # Fills the list with the objects
    for i in range(n):
        ur.append(UR5e(client=client, name=names[i], grip=grip[i], pos=origin[i], dir=dir))

    # Spawns a plane
    __ = Plane(client, dir)


    # ------ Subscriber ------
    # Object spawning into the simulation
    rospy.Subscriber("/object_spawn", String, spawn_cb)

    # Rate
    r = rospy.Rate(100)

    # Control loop
    while not rospy.is_shutdown():

        # For each robot, makes an observation
        for i in range(n):
            ur[i].get_observation()
            
        # Advances the simulation
        p.stepSimulation()
        r.sleep()
        

    