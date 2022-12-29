#! /usr/bin/python3

from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
import rospy

set_point = []
process_value = []
joints_com = []
q = []

for i in range(6):
    set_point.append(0.0)
    process_value.append(0.0)
    q.append(0.0)


def shoulder_pan_listener(data):    
    process_value[0] = data.process_value

def shoulder_lift_listener(data):
    process_value[1] = data.process_value

def elbow_listener(data):
    process_value[2] = data.process_value

def wrist_1_listener(data):
    process_value[3] = data.process_value

def wrist_2_listener(data):
    process_value[4] = data.process_value

def wrist_3_listener(data):
    process_value[5] = data.process_value



def cb_j1(data):
    q[0] = data.data
def cb_j2(data):
    q[1] = data.data
def cb_j3(data):
    q[2] = data.data
def cb_j4(data):
    q[3] = data.data
def cb_j5(data):
    q[4] = data.data
def cb_j6(data):
    q[5] = data.data
    



rospy.init_node("vel", anonymous=True)


rospy.Subscriber('/j1_vel', Float64, cb_j1)
rospy.Subscriber('/j2_vel', Float64, cb_j2)
rospy.Subscriber('/j3_vel', Float64, cb_j3)
rospy.Subscriber('/j4_vel', Float64, cb_j4)
rospy.Subscriber('/j5_vel', Float64, cb_j5)
rospy.Subscriber('/j6_vel', Float64, cb_j6)


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

r = rospy.Rate(10)

g = [0.8, 0.8, 0.8, 1, 0.5, 1]

process_value_msgs = []
for j in range(6):
    process_value_msgs.append(Float64())

while not rospy.is_shutdown():
    for k in range(6):
        process_value_msgs[k].data +=  (q[k] - process_value[k])*g[k]
        joints_com[k].publish(process_value_msgs[k])
    r.sleep()