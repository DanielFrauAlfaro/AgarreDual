#! /usr/bin/python3

from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
import rospy

set_point = []
process_value = []
joints_com = []


for i in range(6):
    set_point.append(0.0)
    process_value.append(0.0)


def shoulder_pan_listener(data):    
    process_value[0] = data.process_value
    set_point[0] = data.set_point + 0.3
def shoulder_lift_listener(data):
    process_value[1] = data.process_value
    set_point[1] = data.set_point
def elbow_listener(data):
    process_value[2] = data.process_value
    set_point[2] = data.set_point
def wrist_1_listener(data):
    process_value[3] = data.process_value
    set_point[3] = data.set_point
def wrist_2_listener(data):
    process_value[4] = data.process_value
    set_point[4] = data.set_point
def wrist_3_listener(data):
    process_value[5] = data.process_value
    set_point[5] = data.set_point

rospy.init_node("vel", anonymous=True)

rospy.Subscriber('/shoulder_pan_joint_position_controller/state', JointControllerState, shoulder_pan_listener)
rospy.Subscriber('/shoulder_lift_joint_position_controller/state', JointControllerState, shoulder_lift_listener)
rospy.Subscriber('/elbow_joint_position_controller/state', JointControllerState, elbow_listener)
rospy.Subscriber('/wrist_1_joint_position_controller/state', JointControllerState, wrist_1_listener)
rospy.Subscriber('/wrist_2_joint_position_controller/state', JointControllerState, wrist_2_listener)
rospy.Subscriber('/wrist_3_joint_position_controller/state', JointControllerState, wrist_3_listener)


joints_com.append(rospy.Publisher('/shoulder_pan_joint_position_controller/command', Float64, queue_size=10))
joints_com.append(rospy.Publisher('/shoulder_lift_joint_position_controller/command', Float64, queue_size=10))
joints_com.append(rospy.Publisher('/elbow_joint_position_controller/command', Float64, queue_size=10))
joints_com.append(rospy.Publisher('/wrist_1_joint_position_controller/command', Float64, queue_size=10))
joints_com.append(rospy.Publisher('/wrist_2_joint_position_controller/command', Float64, queue_size=10))
joints_com.append(rospy.Publisher('/wrist_3_joint_position_controller/command', Float64, queue_size=10))

r = rospy.Rate(50)

process_value_msgs = []
for j in range(6):
    process_value_msgs.append(Float64())

while not rospy.is_shutdown():
    for k in range(6):
        process_value_msgs[k].data =  (set_point[k] - process_value[k])
        joints_com[k].publish(process_value_msgs[k])