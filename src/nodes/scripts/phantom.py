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
scale_grip_3f = 1342.1                # Gripper movement scalation: 255 / 0.19
scale_grip_2f = 5.5

# Origin
or_x = 0.4921                      # Cartesian origin
or_y = 0.1334                       
or_z = 0.4578

or_roll = -1.57                   # Euler orientation origin
or_pitch = -0.00079                   
or_yaw = -3.14 

or_3f = 0.0                    # 3f gripper origin


# Previous positions and orientation in the main loop
prev_x, prev_y, prev_z = or_x, or_y, or_z
prev_roll, prev_pitch, prev_yaw = or_roll, or_pitch, or_yaw

# Previous position of the gripper
prev_grip_pos = 0.0


# Modes and Flags
change = True                       # Flag to indicate a mode change
state = 0                           # Movement state
state_3f = 0                        # 3f gripper movement state


# Actual and previous position and orientation of the Phantom
prev_pose_phantom = Pose()
act_pose_phantom = Pose()

prev_pose_phantom.position.x = 0.0
prev_pose_phantom.position.y = 0.0
prev_pose_phantom.position.z = 0.0
prev_pose_phantom.orientation.x = 0.0
prev_pose_phantom.orientation.y = 0.0
prev_pose_phantom.orientation.z = 0.0

act_pose_phantom.position.x = -1.0
act_pose_phantom.position.y = -1.0
act_pose_phantom.position.z = -1.0
act_pose_phantom.orientation.x = -1.0
act_pose_phantom.orientation.y = -1.0
act_pose_phantom.orientation.z = -1.0


# Force feedback gains (Proportional and derivative)
K = 40                             # Positonal gains
KD = 0.2

K_or = 9                          # Orientational gain
KD_or = 0.06

K_grip_2f = 10.0                    # 2f gripper joints gains
KD_grip_2f = 0.05

K_grip_3f = 0.1                     # 3f gripper joints gains
KD_grip_3f = 0.003

Ke = 70                             # Mode change gain 
Kde = 0.2


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
'''
The robot reference system is different than the one of the Phantom --> 
  - x_robot = z_phantom
  - y_robot = x_phantom
  - z_robot = y_phantom
  
Two variables: 
  - pose: position commands to the controller
  - prev_pose_phantom: previous state of the Phantom
     - .position: prevoius position
     - .orientation: prevoius orientation in Euler angles
  - act_pose_phantom: actual Phantom pose
  
EXPLANATION
    - 1. Gets the current pose of the Phantom
    - 2. If there has not been any changes ...
        - 2.1 ... stores the robot's POSITION according to the Phantom reference system (rescaleted respect the origin)
            The orientation is the last recorded from orientation mode movement

        - 2.2 ... stores the robot's ORIENTTION according to the Phantom reference system (rescaleted respect the origin)
            The position is the last recorded from orientation mode movement
            
        
        - 2.3 ... stores the Phantom positions that corresponds to the gripper joints position of each finger. 
            The system distinguishes between the 2 finger and 3 finger gripper.
'''
def cb(data):
    global pub, pose, scale_x, scale_y, scale_z, scale_roll, scale_pitch, scale_yaw
    global or_x, or_y, or_z, or_roll, or_pitch, or_yaw, or_3f
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    global prev_pose_phantom, act_pose_phantom
    global grip_pos
    global scale_grip
    global state_3f
    global grip
    global prev, interval

    if time.time() - prev > interval:
        prev = time.time()
        # 1. --
        act_pose_phantom.position = data.pose.position
        
        # 2. --
        if not change:
            # 2.1 --
            if state == 0:
                pose.position.x = data.pose.position.z * scale_x + or_x
                pose.position.y = data.pose.position.x * scale_y + or_y
                pose.position.z = data.pose.position.y * scale_z + or_z
                
                pose.orientation.x = prev_roll
                pose.orientation.y = prev_pitch
                pose.orientation.z = prev_yaw
                
                prev_pose_phantom.position = data.pose.position
                
            # 2.2 --    
            elif state == 1:
                pose.position.x = prev_x
                pose.position.y = prev_y
                pose.position.z = prev_z

                pose.orientation.x = data.pose.position.z * scale_roll + or_roll
                pose.orientation.y = data.pose.position.x * scale_pitch + or_pitch
                pose.orientation.z = data.pose.position.y * scale_yaw + or_yaw
                
                prev_pose_phantom.orientation = data.pose.position
                
            # 2.3 --
            elif state == 2 and data.pose.position.y >= 0.0:
                grip_pos = data.pose.position.y * scale_grip
                


# Button 1 callback
def cb_bt1(data):
    global change, state, state_3f
    global pub_change

    if data.buttons[0] == 1:
        state = state + 1       # Advances the state
        change = True           # A change happened
        state_3f = 0

        # If the number exceeds the total state, returns to state 0
        if state > 2:           
            state = 0
        
        # Publishes the change
        pub_change[0].publish(-(state + 1))
        

# Button 2 callback
def cb_bt2(data):
    global change
    global state, state_3f
    
    # If it is in another case, goes to the previous state 
    if data.buttons[0] == 1:
        state = state - 1
        change = True

        # If the number exceeds under the total state, returns to state 2
        if state < 0:
            state = 2

        # Publisher the change
        pub_change[0].publish(-(state + 1))


# Robot cartesian position feedback
def cart_pos(data):
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    global change

    # Gets the position in every mode except in orientation
    if state != 1:
        prev_x = data.position.x
        prev_y = data.position.y
        prev_z = data.position.z

    # Gets the orientation in every mode except in position
    if state != 0:
        prev_roll = data.orientation.x
        prev_pitch = data.orientation.y
        prev_yaw = data.orientation.z


# Gripper position callback
def grip_pos_cb(data):
    global prev_grip_pos

    prev_grip_pos = data.data


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
        elif grip == "2f_140":
            scale_grip = scale_grip_2f

        # Node
        rospy.init_node(name + "_phantom_ctr")

        # ------ Publishers ------
        # Cartesian position command
        pub = rospy.Publisher("/" + name + "/pose", Pose, queue_size=10)
        
        # Phantom cartesian force
        pub_f = rospy.Publisher("/" + name + "/servo_cf", WrenchStamped, queue_size=10)

        # System changes and state
        pub_change.append(rospy.Publisher("/" + name + "/change", Int32, queue_size=10))


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
        
        # Gripper publisher
        pub_grip = rospy.Publisher("/" + name + "/grip_cmd", Float64, queue_size=10)


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

                if state == 0:
                    ex = (prev_pose_phantom.position.x - act_pose_phantom.position.x)
                    ey = (prev_pose_phantom.position.y - act_pose_phantom.position.y)
                    ez = (prev_pose_phantom.position.z - act_pose_phantom.position.z)

                elif state == 1:
                    ex = (prev_pose_phantom.orientation.x - act_pose_phantom.position.x)
                    ey = (prev_pose_phantom.orientation.y - act_pose_phantom.position.y)
                    ez = (prev_pose_phantom.orientation.z - act_pose_phantom.position.z)

                elif state == 2:
                    ex = (0.0 - act_pose_phantom.position.x)
                    ey = (prev_grip_pos / scale_grip - act_pose_phantom.position.y)
                    ez = (0.0 - act_pose_phantom.position.z)

                

                # If the Phantom end effector is within the previous position, 
                #   ends the mode changing state
                if abs(ex) < limit and abs(ey) < limit and abs(ez) < limit:
                    change = False
                    pub_change[0].publish((state + 1))

                        
            # If there has not been a mode changing, the Phantom sends position to the 
            #   robot and applies forces to the operator to restrict his / her movements and
            #   minimize errors. Force is computed by the error 
            else:  

                # Publishes the robot position
                pub.publish(pose)

                # Publishes the gripper position
                pub_grip.publish(grip_pos) 


                if state == 0:
                    ex = (prev_y - pose.position.y)
                    ey = (prev_z - pose.position.z)
                    ez = (prev_x - pose.position.x)
                
                elif state == 1:
                    ex = (prev_pitch - pose.orientation.y)

                    # Corrects the orientation so it remains inside the [-pi,pi] interval    
                    if prev_yaw < 0.0:
                        prev_yaw = 2*pi + prev_yaw
                    if pose.orientation.z < 0.0:
                        pose.orientation.z = 2*pi + pose.orientation.z

                    ey = (prev_yaw - pose.orientation.z)
                    ez = (prev_roll - pose.orientation.x)

                # On the gripper mode, the Phantom can not go to negative positions
                elif state == 2:
                    ex = (0 - act_pose_phantom.position.x) * 10
                    ez = (0 - act_pose_phantom.position.z) * 10
                    ex0 = (0 - act_pose_phantom.position.x) * 10
                    ey0 = (0 - act_pose_phantom.position.y) * 10
                    ez0 = (0 - act_pose_phantom.position.z) * 10

                    ey = (prev_grip_pos - grip_pos) 
                    
                    # If some coordinate is negative, the error is the one between the 
                    #   origin and the actual position
                    if act_pose_phantom.position.y < 0.0:
                        ey = ey0

                        if grip == "3f":
                            ey = ey0 * 50

            # Depending on the mode, some gains are applied
            k = 0
            kd = 0

            if change:
                k = Ke
                kd = Kde

            elif state == 0:
                k = K
                kd = KD
            
            elif state == 1:
                k = K_or
                kd = KD_or
            
            elif state == 2:
                if grip == "2f_140":
                    k = K_grip_2f
                    kd = KD_grip_2f
            
                elif grip == "3f":
                    k = K_grip_3f
                    kd = KD_grip_3f

                    ez = ez * 50
                    ex = ex * 50
            

            # Computes the forces
            wrench.wrench.force.x = ex * k - ex / (time.time() - t) * kd
            wrench.wrench.force.y = ey * k - ey / (time.time() - t) * kd
            wrench.wrench.force.z = ez * k - ez / (time.time() - t) * kd

            # Upwards constant force if there is not any change
            if not change and state == 0:
                wrench.wrench.force.y = wrench.wrench.force.y + 0.9

            # Wrench publisher
            pub_f.publish(wrench)

            # Updates time
            t = time.time()

            r.sleep()