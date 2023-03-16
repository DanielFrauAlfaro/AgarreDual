#!/usr/bin/env python3

import pybullet as p
import os


class Plane:
    def __init__(self, client, dir):
        # Import land plane
        p.loadURDF(fileName=dir + '/models/urdf/simpleplane.urdf',
                              basePosition=[0, 0, 0], 
                              physicsClientId=client,
                              useFixedBase=1)