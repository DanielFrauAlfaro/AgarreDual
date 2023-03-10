#!/usr/bin/env python3

import pybullet as p
import sys
import rospy
from ur5_class import UR5e
from std_msgs.msg import String

client = 0

def spawn_cb(data):
    global client
    
    s = data.data.split()
    name = s[0]
    pos = [float(s[1]), float(s[2]), float(s[3])]
    orient = p.getQuaternionFromEuler(eulerAngles=[float(s[4]), float(s[5]), float(s[6])],
                                      physicsClientId=client)
    
    # p.loadURDF(fileName=s[-1] + "/src/object_models/urdf/" + name + "/model.urdf",
    #            basePosition=pos,
    #            baseOrientation=orient)
    

if __name__ == "__main__":
    rospy.init_node("master_pybullet")
    client = p.connect(p.GUI)

    n = int(sys.argv[1])
    names = [sys.argv[2], sys.argv[3]]
    origin = [[float(sys.argv[5]), float(sys.argv[7]), float(sys.argv[9])], 
              [float(sys.argv[11]), float(sys.argv[13]), float(sys.argv[15])]]
    grip = [sys.argv[16], sys.argv[17]]
    dir = sys.argv[18]

    ur = []

    for i in range(n):
        ur.append(UR5e(client=client, name=names[i], grip=grip[i], pos=origin[i], dir=dir))

    rospy.Subscriber("/object_spawn", String, spawn_cb)

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        for i in range(n):
            ur[i].get_observation()
            
        p.stepSimulation()
        r.sleep()
        

    