#!/usr/bin/env python3

import rospy
import sys
from gripper_controller import GripperController


# ---- Main ----
if __name__ == "__main__":

    # Gets arguments
    name = sys.argv[1]
    model = sys.argv[2]

    # Node
    rospy.init_node(name + "_gripper_controller")

    # According to the model, creates the finger controller
    if model == "3f":
        __ = GripperController(name, model)

    
    elif model == "2f_140":
        __ = GripperController(name, model)

    # Spin
    rospy.spin()