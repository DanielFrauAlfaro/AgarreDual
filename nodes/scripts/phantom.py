#! /usr/bin/python3

from std_msgs.msg import Float64, Int32, Float32
from geometry_msgs.msg import PoseStamped, WrenchStamped, Pose
from sensor_msgs.msg import Joy
import rospy
import sys
import time
from math import pi

'''
x_limit: 0.09 - -0.07           * 2
y_limit: 0.2                    * 2
z_limit: 0.19  - -0.075         * 0.4 // *7
'''

# Messages
pose = Pose()                      # End effector position
wrench = WrenchStamped()           # Phantom wrench

# Joint position of the gripper
grip_pos = 0.0

# Movements scalation
scale_x = 2                        # Cartesian movement scalation
scale_y = 2
scale_z = 3.5

scale_roll = 12                    # Euler orientation movement scalation
scale_pitch = 12
scale_yaw = 12

scale_grip = 0
scale_grip_3f = 1342.1             # Gripper movement scalation: 255 / 0.19
scale_grip_2f = 1342.1             # 990

# Origin
or_x = 0.4921                      # Cartesian origin
or_y = 0.1334                       
or_z = 0.4578

or_roll = -1.57                   # Euler orientation origin
or_pitch = -0.00079                   
or_yaw = -3.14 

or_3f = 0.0                    # 3f gripper origin


# Previous position of the gripper
prev_grip_pos = 0.0


# Modes and Flags
change = True                       # Flag to indicate a mode change
state = 0                           # Movement state


# Actual and previous position and orientation of the Phantom
act_pose_phantom = Pose()

act_pose_phantom.position.x = -1.0
act_pose_phantom.position.y = -1.0
act_pose_phantom.position.z = -1.0
act_pose_phantom.orientation.x = -1.0
act_pose_phantom.orientation.y = -1.0
act_pose_phantom.orientation.z = -1.0

prev_poses_phantom = [[0,0,0], [0,0,0], [0,0,0]]  


# Previous positions and orientation in the main loop
poses = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
prevs = [[or_x, or_y, or_z], [or_roll, or_pitch, or_yaw], [0, 0, 0]]


# Force feedback gains (Proportional and derivative)
K = 40                             # Positonal gains
KD = 0.2

K_or = 9                          # Orientational gain
KD_or = 0.06

K_grip_2f = 0.1                    # 2f gripper joints gains
KD_grip_2f = 0.003

K_grip_3f = 0.1                     # 3f gripper joints gains
KD_grip_3f = 0.003

Ke = 70                             # Mode change gain 
Kde = 0.2

# Gains lists
Ks = [K, K_or]
Kds = [KD, KD_or]

# Depending on the mode, some gains are applied
k = 0
kd = 0

# Detection umbral
limit = 0.005


# Names
name = "ur5_2"
grip = "3f"


# Publisher for the mode change
pub_change = []


# Intervals and times
prev = time.time()
interval = 0.07



# Phantom cartesian position callback
def cb(data):
    global pub, pose, scale_x, scale_y, scale_z, scale_roll, scale_pitch, scale_yaw
    global or_x, or_y, or_z, or_roll, or_pitch, or_yaw, or_3f
    global act_pose_phantom
    global grip_pos
    global scale_grip
    global grip
    global prev, interval

    if time.time() - prev > interval:
        prev = time.time()

        # Phantom current position
        act_pose_phantom.position = data.pose.position


        # If the Phantom is not in change, gathers the Phantom position and remaps it. 
        #    Also saves the current position in UR5 reference system
        if not change:
            
            if state == 0:
                pose.position.x = data.pose.position.z * scale_x + or_x
                pose.position.y = data.pose.position.x * scale_y + or_y
                pose.position.z = data.pose.position.y * scale_z + or_z
                
                pose.orientation.x = prevs[1][0]
                pose.orientation.y = prevs[1][1]
                pose.orientation.z = prevs[1][2]


                poses[state][0] = pose.position.x
                poses[state][1] = pose.position.y
                poses[state][2] = pose.position.z

                
            # Orientation   
            elif state == 1:
                pose.position.x = prevs[0][0]
                pose.position.y = prevs[0][1]
                pose.position.z = prevs[0][2]

                pose.orientation.x = data.pose.position.z * scale_roll + or_roll
                pose.orientation.y = data.pose.position.x * scale_pitch + or_pitch
                pose.orientation.z = data.pose.position.y * scale_yaw + or_yaw

                if pose.orientation.z < 0.0:
                    pose.orientation.z = 2*pi + pose.orientation.z

                poses[state][0] = pose.orientation.x
                poses[state][1] = pose.orientation.y
                poses[state][2] = pose.orientation.z

                
            # Gripper
            elif state == 2 and data.pose.position.y >= 0.0:
                grip_pos = data.pose.position.y * scale_grip

                poses[state][2] = grip_pos

                poses[state][0] = act_pose_phantom.position.z
                poses[state][1] = act_pose_phantom.position.x

                


# Button 1 callback
def cb_bt1(data):
    global change, state
    global pub_change, k, kd

    if data.buttons[0] == 1:
        state = state + 1       # Advances the state
        change = True           # A change happened

        # If the number exceeds the total state, returns to state 0
        if state > 2:           
            state = 0
        
        # Publishes the change
        pub_change[0].publish(-(state + 1))
        k = Ke
        kd = Kde


# Button 2 callback
def cb_bt2(data):
    global change
    global state, pub_change, k, kd
    
    # If it is in another case, goes to the previous state 
    if data.buttons[0] == 1:
        state = state - 1
        change = True

        # If the number exceeds under the total state, returns to state 2
        if state < 0:
            state = 2

        # Publisher the change
        pub_change[0].publish(-(state + 1))
        k = Ke
        kd = Kde


# Robot cartesian position feedback
def cart_pos(data):
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    global change

    # Gets the position in every mode except in orientation
    if state != 1:
        prevs[0][0] = data.position.x
        prevs[0][1] = data.position.y
        prevs[0][2] = data.position.z

    # Gets the orientation in every mode except in position
    if state != 0:
        prevs[1][0] = data.orientation.x
        prevs[1][1] = data.orientation.y
        prevs[1][2] = data.orientation.z


        if prevs[1][2] < 0.0:
            prevs[1][2] = 2*pi + prevs[1][2]


# Gripper position callback
def grip_pos_cb(data):
    global prev_grip_pos

    # prev_grip_pos = data.data
    prevs[2][2] = data.data - 3


# ---- Main ----
if __name__ == "__main__":

    # If there are arguments, starts the function
    if len(sys.argv) > 0:
        
        # Initializes the robot name and gripper type
        name = sys.argv[1]
        grip = sys.argv[2] 
        
        # Adjusts gripper scales
        if grip == "3f":
            scale_grip = scale_grip_3f
            Ks.append(K_grip_3f)
            Kds.append(KD_grip_3f)

        elif grip == "2f_140":
            scale_grip = scale_grip_2f
            Ks.append(K_grip_2f)
            Kds.append(KD_grip_2f)
            

        # Node
        rospy.init_node(name + "_phantom_ctr")

        # ------ Publishers ------
        # Cartesian position command
        pub = rospy.Publisher("/" + name + "/pose", Pose, queue_size=10)
        
        # Phantom cartesian force
        pub_f = rospy.Publisher("/" + name + "/servo_cf", WrenchStamped, queue_size=10)

        # System changes and state
        pub_change.append(rospy.Publisher("/" + name + "/change", Int32, queue_size=10))

        # Gripper publisher
        pub_grip = rospy.Publisher("/" + name + "/grip_cmd", Float64, queue_size=10)


        # ------ Subscribers -----
        # Phantom cartesian position
        rospy.Subscriber("/" + name + "/measured_cp", PoseStamped,cb)
        
        # Phantom buttons
        rospy.Subscriber("/" + name + "/button1", Joy, cb_bt1)
        rospy.Subscriber("/" + name + "/button2", Joy, cb_bt2)
        
        # Robot cartesian position feedback
        rospy.Subscriber('/' + name + '/cart_pos', Pose, cart_pos)
        
        # Gripper postion feedback
        rospy.Subscriber("/" + name + "/grip_state", Float32, grip_pos_cb)
        
        k = Ke
        kd = Kde

        # Time
        t = time.time()

        # Rate
        r = rospy.Rate(18)

        # Initializes errors
        ex = 0
        ey = 0
        ez = 0
        ex0 = 0
        ey0 = 0
        ez0 = 0


        # Control loop: sends catesian commands to the robot controller and applies forces
        #   to the manipulator according to the error between the robot position and 
        #   the Phantom one
        while not rospy.is_shutdown():
            
            # If there has been a change, computes the error the previous position of that 
            #   mode
            if change:

                ex = (prev_poses_phantom[state][0] - act_pose_phantom.position.x)
                ey = (prev_poses_phantom[state][1] - act_pose_phantom.position.y)
                ez = (prev_poses_phantom[state][2] - act_pose_phantom.position.z) 
                    

                # If the Phantom end effector is within the previous position, 
                #   ends the mode changing state
                if abs(ex) < limit and abs(ey) < limit and abs(ez) < limit:
                    change = False
                    pub_change[0].publish((state + 1))
                    
                    # Changes gains
                    k = Ks[state]
                    kd = Kds[state]

                        
            # If there has not been a mode changing, the Phantom sends position to the 
            #   robot and applies forces to the operator to restrict his / her movements and
            #   minimize errors. Force is computed through the error 
            else:  
                
                # Y error respect to zero
                ey0 = (0 - act_pose_phantom.position.y) * 80

                ex = (prevs[state][1] - poses[state][1])
                ey = (prevs[state][2] - poses[state][2])
                ez = (prevs[state][0] - poses[state][0])

                # If state is position or orientation, publishes position
                if state < 2:
                    pub.publish(pose)

                # If state is gripper, publishes gripper position
                else:
                    
                

                    pub_grip.publish(grip_pos) 

                    # On the gripper mode, the Phantom can not go to negative positions
                    if act_pose_phantom.position.y < 0.0:
                        ey = ey0

                        ey = ey0 * 10
                    
                    ez = ez * 800
                    ex = ex * 800


            # Computes the forces
            T = (time.time() - t)

            wrench.wrench.force.x = ex * k - ex / T * kd
            wrench.wrench.force.y = ey * k - ey / T * kd + int(not change) * 0.9 * 0.0
            wrench.wrench.force.z = ez * k - ez / T * kd


            # Wrench publisher
            pub_f.publish(wrench)


            # Updates time
            t = time.time()
            r.sleep()