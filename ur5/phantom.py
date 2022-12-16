#! /usr/bin/python3

from geometry_msgs.msg import PoseStamped, Pose
import rospy

# Button: sensor_msgs/Joy /arm/button1    /arm/button2

# Wrench: geometry_msgs/WrenchStamped   /arm/servo_cf
# Fuerza: y = 0.9
'''
x_limit: 0.09 - -0.07
y_limit: 0.2
z_limit: 0.19  - -0.075
'''

rospy.init_node("phantom_ctr")
pub = rospy.Publisher("/pose", Pose, queue_size=10)

pose = Pose()

def cb(data):
    global pub, pose

    pose.position.x = data.pose.position.z + 0.5095
    pose.position.y = data.pose.position.x + 0.1334
    pose.position.z = data.pose.position.y + 0.7347
    
    pose.orientation.x = 1.57225399 
    pose.orientation.y = 1.07079575
    pose.orientation.z = -0.001661

rospy.Subscriber("/arm/measured_cp", PoseStamped,cb)

 
r = rospy.Rate(10)
while True:
    pub.publish(pose)
    r.sleep()
