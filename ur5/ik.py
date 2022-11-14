from spatialmath import *
import roboticstoolbox as rtb
from math import pi
import matplotlib as plt
import cv2 as cv
import numpy as np

ur5 = rtb.models.DH.UR5()

q = [pi, -1.5, 1 , 0.0, 0.0, 0.0]
T = ur5.fkine(q)

sol = ur5.ikine_LM(T,q0 = q)


# traj = rtb.jtraj(ur5.qr, q, 100)
# ur5.plot(traj.q)

# Ts = rtb.tools.trajectory.ctraj(T0, T1, len(t))

# sol = ur5.ikine_LM(Ts)   
# ur5.plot(sol.q)

#sol = ur5.ikine_LM(T, ilimit=1000, rlimit=1000)  
#print(sol)

# ur5.teach()

ur5.DH