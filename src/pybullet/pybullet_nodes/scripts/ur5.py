#!/usr/bin/env python3

import pybullet as p
import sys
import rospy
from ur5_class import UR5e
from plane import Plane
from std_msgs.msg import String

# Client
client = 0


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
    

# Main
if __name__ == "__main__":

    # Node 
    rospy.init_node("master_pybullet")
    
    # Starts the simulation and applies gravity
    client = p.connect(p.GUI)
    p.setGravity(0, 0, -10, physicsClientId=client) 

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
        

    