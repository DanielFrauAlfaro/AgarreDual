#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import collections
from spatialmath import SE3
import roboticstoolbox as rtb
from math import pi
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

# Class for the UR5
class UR5e:
    def __init__(self, client, grip="3f", pos = [0,0,0], name="ur5_1", dir=""):
        self.client = client
        # rospy.init_node(name + "_controller")

        self.name = name

        # Internal UR5 DH model
        self.__ur5 = rtb.DHRobot([
            rtb.RevoluteDH(d=0.1625, alpha=pi/2.0),
            rtb.RevoluteDH(a=-0.425),
            rtb.RevoluteDH(a = -0.3922),
            rtb.RevoluteDH(d = 0.1333, alpha=pi/2.0),
            rtb.RevoluteDH(d = 0.0997, alpha=-pi/2.0),
            rtb.RevoluteDH(d = 0.0996)
        ], name="UR5e")
    
        self.__ur5.base = SE3.RPY(0,0,-pi)      # Rotate robot base so it matches Gazebo model
        self.__ur5.tool = SE3(0.0, 0.0, 0.03)

        # Client ID of the Pybullet server
        


        # Load the UR5 URDF according to 'grip' value
        self.ur5 = 0
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

        if grip == "3f":
            self.ur5 = p.loadURDF(fileName=dir + '/models/urdf/ur5e_3f.urdf',
                              basePosition=pos, 
                              physicsClientId=self.client, 
                              useFixedBase=1)

            # Gripper joint names
            gripperJoints = ["palm_finger_1_joint", "finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"]

            # Gripper 3f initial position
            self.gripper = [0.0, 0.0, 0.0, 0]


        elif grip == "2f_140":
             
            self.ur5 = p.loadURDF(fileName=dir + '/models/urdf/ur5e_2f.urdf',
                              basePosition=pos, 
                              physicsClientId=self.client, 
                              useFixedBase=1)
            
            # Gripper joint names
            gripperJoints = ["finger_joint"]
            
            # Gripper 2f initial position
            self.gripper = [0.0]



        # Name of the UR5 joints (all joints are included on the same body)
        controlJoints = ["shoulder_pan_joint","shoulder_lift_joint",
                        "elbow_joint", "wrist_1_joint",
                        "wrist_2_joint", "wrist_3_joint"]
        
        # Get info from joints and enable joint torque sensor
        self.joints = []
        self.ur5_joints_id = []
        self.gripper_joints_id = []
        numJoints = p.getNumJoints(self.ur5)
        
        # Iterates for each joint
        for i in range(numJoints):
            info = p.getJointInfo(self.ur5, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2]

            controllable = (jointType != p.JOINT_FIXED)

            # If a joint is controllable ...
            if controllable:
                # ... enables torque sensors, ...
                p.enableJointForceTorqueSensor(bodyUniqueId=self.ur5, 
                                               jointIndex=jointID, 
                                               enableSensor=1,
                                               physicsClientId=self.client)

                # ... saves its properties, ...
                info = [jointID,jointName,jointType,controllable]
                self.joints.append(info)

                # ... saves the IDs of the UR5 joints, ...
                if jointName in controlJoints:
                    self.ur5_joints_id.append(jointID)
                
                # ... saves the IDs of the Gripper joints and ...
                if jointName in gripperJoints:
                    self.gripper_joints_id.append(jointID)


        # Setups mimic joints for the fingers
        if grip == "3f":
            self.gripperControl3f('middle')
            self.gripperControl3f('1')
            self.gripperControl3f('2')
            self.palmControl()

        elif grip == "2f_140":
            self.gripperControl2f()

        # Starting joint positions for the robot and the gripper
        self.q = [0, -1.5, 1.57 , -1.57, -1.57, 0.0]
        
        # Brings the robot to a starting position
        self.apply_action(self.q + self.gripper)
        
        for i in range(22):
            p.stepSimulation(self.client)


        # Publishers and subscribers
        rospy.Subscriber("/" + self.name + "/pose", Pose, self.pose_cb)
        self.pub_state = rospy.Publisher("/" + self.name + "/joint_states", JointState, queue_size=10)
        self.torque_pub = []

        for i in controlJoints:
            self.torque_pub.append(rospy.Publisher("/" + self.name + "/" + i + "_torque_sensor", WrenchStamped, queue_size=10))

        if grip == "2f_140":
            rospy.Subscriber("/" + name + "/gripper/command", Float64, self.grip_2f_cb )

        else:
            rospy.Subscriber("/" + name + "/finger_1_joint_1_controller/command", Float64, self.grip_3f_1_cb )
            rospy.Subscriber("/" + name + "/finger_2_joint_1_controller/command", Float64, self.grip_3f_2_cb )
            rospy.Subscriber("/" + name + "/finger_middle_joint_1_controller/command", Float64, self.grip_3f_mid_cb )
            rospy.Subscriber("/" + name + "/palm_finger_1_joint_controller/command", Float64, self.grip_3f_palm_cb )
        
        self.t_q = WrenchStamped()

        self.j_state = JointState()
        self.j_state.name = controlJoints + gripperJoints
        self.j_state.position = self.q + self.gripper
        self.j_state.velocity = [0,0,0,0,0,0] +  self.gripper
        self.j_state.effort = [0,0,0,0,0,0] + self.gripper

        self.pose = Pose()

        x = 0.5137                       
        y = 0.1334                       
        z = 0.4397

        roll = -2.296 
        pitch = 0.0 
        yaw = -3.03

        T = SE3(x, y, z)
        T_ = SE3.RPY(roll, pitch, yaw, order='yxz')
        
        self.T = T * T_


    # Moves the robot to a desired position
    def apply_action(self, action):
        # Decodes the action in robot joint position, gripper position and palm state
        q = action[0:6]

        # Assigns the action to the internal values of the robot
        self.q = q
        
        # UR5 control
        p.setJointMotorControlArray(bodyUniqueId=self.ur5, 
                                    jointIndices=self.ur5_joints_id, 
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=self.q,
                                    physicsClientId=self.client)

    
    # Returns observation of the robot state
    def get_observation(self):
        # UR5 joint values
        cont = 0

        for i in self.ur5_joints_id:
            aux = p.getJointState(bodyUniqueId=self.ur5, 
                                jointIndex=i,
                                physicsClientId=self.client)

            self.j_state.position[cont] = aux[0]
            self.j_state.velocity[cont] = aux[1]

            self.t_q.wrench.torque.y = aux[2][-1]
            self.t_q.wrench.torque.z = aux[2][-1]
            self.torque_pub[cont].publish(self.t_q)

            cont = cont + 1

        # Gripper joint values
        for i in self.gripper_joints_id:
            aux = p.getJointState(bodyUniqueId=self.ur5, 
                                jointIndex=i,
                                physicsClientId=self.client)
            
            self.j_state.position[cont] = aux[0]
            self.j_state.velocity[cont] = aux[1]

            cont = cont + 1
        
        
        self.pub_state.publish(self.j_state)
    
        
            
        
####################################################################
    
    # Apply constraints to each finger - 2f gripper
    def setup_mimic_joints_2f(self, mimic_parent_name, mimic_children_names):
        keys = list(mimic_children_names.keys())
        for i in self.joints:
            for j in mimic_children_names:

                if mimic_parent_name == i[1]:
                    parent_id = i[0]
                elif i[1] == keys[0]:
                    child1 = i[0]
                elif i[1] == keys[1]:
                    child2 = i[0]
                elif i[1] == keys[2]:
                    child3 = i[0]
                elif i[1] == keys[3]:
                    child4 = i[0]
                elif i[1] == keys[4]:
                    child5 = i[0]

        mimic_parent_id = parent_id
        mimic_child_multiplier = {child1, child2, child3, child4, child5}


        c = []
        for joint_id in mimic_child_multiplier:
            c.append(p.createConstraint(self.ur5, mimic_parent_id, self.ur5, joint_id, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0]))

        cont = 0
        for i in mimic_children_names:
            p.changeConstraint(c[cont], gearRatio=-mimic_children_names[i], maxForce=100, erp=1)  # Note: the mysterious `erp` is of EXTREME importance
            cont=cont+1

        return mimic_parent_id

    # Define mimic joints - 2f gripper
    def gripperControl2f(self):

        mimic_parent_name = 'finger_joint'
        mimic_children_names = {'left_inner_finger_joint': 1,
                                'left_inner_knuckle_joint': -1,
                                'right_outer_knuckle_joint': -1,
                                'right_inner_finger_joint': 1,  
                                'right_inner_knuckle_joint': -1}

        mimic_parent_id = self.setup_mimic_joints_2f(self.ur5, mimic_parent_name, mimic_children_names)

        return mimic_parent_id

    # Define mimic joints - 3f gripper
    def gripperControl3f(self, n):
        # Establish parent and children names for the mimic configuration
        mimic_parent_name = 'finger_' + n + '_joint_1'
        mimic_children_names = {'finger_' + n + '_joint_2': 1,
                                'finger_' + n + '_joint_3': 1}

        mimic_parent_id = self.setup_mimic_joints_3f(self.ur5, mimic_parent_name, mimic_children_names)
      
    # Apply constraints to each finger - 3f gripper
    def setup_mimic_joints_3f(self, mimic_parent_name, mimic_children_names):

        keys = list(mimic_children_names.keys())

        for i in self.joints:
            for j in mimic_children_names:
                if mimic_parent_name == i[1]:
                    parent_id = i[0]
                elif i[1] == keys[0]:
                    child1 = i[0]
                elif i[1] == keys[1]:
                    child2 = i[0]


        mimic_parent_id = parent_id
        mimic_child_multiplier = {child1, child2}


        c = []
        for joint_id in mimic_child_multiplier:
            c.append(p.createConstraint(self.ur5, mimic_parent_id, self.ur5, joint_id, jointType=p.JOINT_GEAR, jointAxis=[0, 0, 1], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0]))

        cont = 0
        for i in mimic_children_names:
            p.changeConstraint(c[cont], gearRatio=-mimic_children_names[i], maxForce=100, erp=1)  # Note: the mysterious `erp` is of EXTREME importance
            cont=cont+1

    # Apply constraint to the palm joints - 3f gripper
    def palmControl(self):
        # Parent and child names
        mimic_parent_name = 'palm_finger_1_joint'
        mimic_children_name = 'palm_finger_2_joint'
        
        # Iterates to find the parent and child IDs
        for i in self.joints:
            for j in mimic_children_name:
                if mimic_parent_name == i[1]:
                    mimic_parent_id = i[0]
                elif i[1] == mimic_children_name:
                    joint_id = i[0]
        
        # Applies and modifies the constraint
        c = p.createConstraint(self.ur5, mimic_parent_id, self.ur5, joint_id, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
        p.changeConstraint(c, gearRatio=1, maxForce=100, erp=1)  # Note: the mysterious `erp` is of EXTREME importance
    
    #####################################################################

    # Return the client and robot's IDs
    def get_ids(self):
        return self.client, self.ur5
    

    def grip_3f_1_cb(self, data):
        p.setJointMotorControl2(bodyIndex=self.ur5, 
                                jointIndex=self.gripper_joints_id[1], 
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=data.data,
                                physicsClientId=self.client)

    def grip_3f_2_cb(self, data):
        p.setJointMotorControl2(bodyIndex=self.ur5, 
                                jointIndex=self.gripper_joints_id[2], 
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=data.data,
                                physicsClientId=self.client)

    def grip_3f_mid_cb(self, data):
        p.setJointMotorControl2(bodyIndex=self.ur5, 
                                jointIndex=self.gripper_joints_id[3], 
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=data.data,
                                physicsClientId=self.client)
    def grip_3f_palm_cb(self, data):
        p.setJointMotorControl2(bodyIndex=self.ur5, 
                                jointIndex=self.gripper_joints_id[0], 
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=data.data,
                                physicsClientId=self.client)
    
    def grip_2f_cb(self, data):
        
        p.setJointMotorControl2(bodyIndex=self.ur5, 
                                jointIndex=self.gripper_joints_id[0], 
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=data.data,
                                physicsClientId=self.client)

    def pose_cb(self, data):
        x = data.position.x                                 
        y = data.position.y
        z = data.position.z
            
        roll = data.orientation.x
        pitch = data.orientation.y
        yaw = data.orientation.z
        
        T = SE3(x, y, z)
        T_ = SE3.RPY(roll, pitch, yaw, order='yxz')
        
        self.T = T * T_

        q = self.__ur5.ikine_LMS(self.T,q0 = self.j_state.position[0:6])       # Inversa: obtiene las posiciones articulares a través de la posición    
        self.apply_action(q.q)