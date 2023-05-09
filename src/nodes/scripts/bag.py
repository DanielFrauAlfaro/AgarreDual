#! /usr/bin/python3

import sys
import rospy
import rosbag
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import JointState


info = [[Pose(), Pose(), Float32(), Float64(), JointState()],
        [Pose(), Pose(), Float32(), Float64(), JointState()]]

def cart_cb(data):
    global info

    info[0][0] = data

def cart_cb2(data):
    global info

    info[1][0] = data

def pose_cb(data):
    global info

    info[0][1] = data

def pose_cb2(data):
    global info

    info[1][1] = data

def grip_state_cb(data):
    global info

    info[0][2] = data

def grip_state_cb2(data):
    global info

    info[1][2] = data

def grip_cmd_cb(data):
    global info

    info[0][3] = data

def grip_cmd_cb2(data):
    global info

    info[1][3] = data

def joint_cb(data):
    global info

    info[0][4] = data

def joint_cb2(data):
    global info

    info[1][4] = data


# ---- Main ----
if __name__ == "__main__":

    n = sys.argv[1]
    name1 = sys.argv[2]
    name2 = sys.argv[3]
    name_bag = sys.argv[4]

    bag = rosbag.Bag(name_bag + ".bag", "w")

    rospy.init_node("node_bag")

    
    rospy.Subscriber("/" + name1 + "/cart_pos", Pose, cart_cb)
    rospy.Subscriber("/" + name1 + "/pose", Pose, pose_cb)

    rospy.Subscriber("/" + name1 + "/grip_state", Float32, grip_state_cb)
    rospy.Subscriber("/" + name1 + "/grip_cmd", Float64, grip_cmd_cb)

    rospy.Subscriber("/" + name1 + "/joint_states", JointState, joint_cb)

    if n == "2":
        rospy.Subscriber("/" + name2 + "/cart_pos", Pose, cart_cb2)
        rospy.Subscriber("/" + name2 + "/pose", Pose, pose_cb2)

        rospy.Subscriber("/" + name2 + "/grip_state", Float32, grip_state_cb2)
        rospy.Subscriber("/" + name2 + "/grip_cmd", Float64, grip_cmd_cb2)

        rospy.Subscriber("/" + name2 + "/joint_states", JointState, joint_cb2)

    N = int(n)
    topic_names = ["cart_pos", "pose", "grip_state", "grip_cmd", "joint_states"]
    names = [name1, name2]

    # ---- Infinite loop ----
    while not rospy.is_shutdown():
        
        for i in range(N):
            for j in range(len(topic_names)):
                print("/" + names[i] + "/" + topic_names[j])
                print(info[i][j])
                bag.write("/" + names[i] + "/" + topic_names[j], info[i][j])

       
        

    bag.close()
