#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Float64, Float32
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input, _Robotiq2FGripper_robot_output
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput, Robotiq3FGripperRobotInput


# Messages
msg_2f = _Robotiq2FGripper_robot_output.Robotiq2FGripper_robot_output()
msg_2f.rACT = 1
msg_2f.rGTO = 1

msg_3f = Robotiq3FGripperRobotOutput()
msg_3f.rACT = 1
msg_3f.rGTO = 1

# Publishers commands
pub_cmd = []

# States list
pub_state = []

# Model name
model = ""

# Start the node
start_node = False

# Position command callback
def cmd_cb(data):
    global model, pub_cmd, msg_2f, msg_3f
    
    msg = []

    # According to the model, creates a message
    if model == "2f_140":
        msg_2f.rPR = int(data.data)
        msg.append(msg_2f)

    elif model == "3f":
        msg_3f.rPRA = int(data.data)
        msg.append(msg_3f)


    # Publishes the message
    pub_cmd[0].publish(msg[0])


# Joint state callback
def state_cb(data):
    global model, pub_state, start_node
    
    
    if not start_node:
        start_node = True

    else:
        # Depending on the model, decodes the message to get the state
        if model == "2f_140":
            state = data.gPO

            msg = Float32()
            msg.data = float(state)


        elif model == "3f":
            state = data.gPOA

            if data.gPOB < state:
                state = data.gPOB
            
            if data.gPOC < state:
                state = data.gPOC

            msg = Float32()
            msg.data = float(state)

        # Publishes the state
        pub_state[0].publish(msg)



# Output es el cmd

# ---- Main ----
if __name__ == "__main__":

    # Gets arguments
    name = sys.argv[1]
    model = sys.argv[2]

    # Node
    rospy.init_node(name + "_gripper_controller")

    # --- Publishers ---
    # Publisher of the gripper state
    pub_state.append(rospy.Publisher("/" + name + "/grip_state", Float32, queue_size=10))

    # --- Subscribers ---
    # Topic for the gripper commands
    rospy.Subscriber("/" + name + "/grip_cmd", Float64, cmd_cb)


    # According to the gripper model, subscribes and publishes to the 3f or 2f topics
    if model == "3f":
        pub_cmd.append(rospy.Publisher("/" + name + "/Robotiq3FGripperRobotOutput", Robotiq3FGripperRobotOutput, queue_size=10))
        rospy.Subscriber("/" + name + "/Robotiq3FGripperRobotInput", Robotiq3FGripperRobotInput, state_cb)

    elif model == "2f_140":
        pub_cmd.append(rospy.Publisher("/" + name + "/Robotiq2FGripperRobotOutput", _Robotiq2FGripper_robot_output.Robotiq2FGripper_robot_output, queue_size=10))
        rospy.Subscriber("/" + name + "/Robotiq2FGripperRobotInput", _Robotiq2FGripper_robot_input.Robotiq2FGripper_robot_input, state_cb)
    

    # Spin
    rospy.spin()