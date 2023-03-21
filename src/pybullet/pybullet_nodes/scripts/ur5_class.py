#!/usr/bin/env python3

import pybullet as p
import pybullet_data
from spatialmath import SE3
import roboticstoolbox as rtb
from math import pi
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

# Class for the UR5
class UR5e:
    def __init__(self, client, grip="3f", pos = [0,0,0], name="ur5_1", dir=""):
        self.client = client

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
            gripper_gripperJoints = ["gripper_palm_finger_1_joint", "gripper_finger_1_joint_1", "gripper_finger_2_joint_1", "gripper_finger_middle_joint_1"]

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
            gripper_gripperJoints = ["finger_joint"]


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
            self.gripperControl3f('2')
            self.gripperControl3f('1')
            self.palmControl()

        elif grip == "2f_140":
            self.gripperControl2f()

        # Starting joint positions for the robot and the gripper
        self.q = [0, -1.57, 1.57 , -1.57, -1.57, 0.0]
        
        # Gets the robot to a starting position
        self.apply_action(self.q + self.gripper)
        
        # Waits for the robot to be in position
        for i in range(22):
            p.stepSimulation(self.client)


        # ------ Publishers ------
        # Publish joint states
        self.pub_state = rospy.Publisher("/" + self.name + "/joint_states", JointState, queue_size=10)


        # ------ Subscribers ------
        # Reference robot cartesian pose
        self.__joints_com = []
        self.__joints_com.append(rospy.Subscriber('/' + self.name + '/shoulder_pan_joint_position_controller/command', Float64, self._joint_1_cb))
        self.__joints_com.append(rospy.Subscriber('/' + self.name + '/shoulder_lift_joint_position_controller/command', Float64, self._joint_2_cb))
        self.__joints_com.append(rospy.Subscriber('/' + self.name + '/elbow_joint_position_controller/command', Float64, self._joint_3_cb))
        self.__joints_com.append(rospy.Subscriber('/' + self.name + '/wrist_1_joint_position_controller/command', Float64, self._joint_4_cb))
        self.__joints_com.append(rospy.Subscriber('/' + self.name + '/wrist_2_joint_position_controller/command', Float64, self._joint_5_cb))
        self.__joints_com.append(rospy.Subscriber('/' + self.name + '/wrist_3_joint_position_controller/command', Float64, self._joint_6_cb))


        # Gripper position callbacks
        if grip == "2f_140":
            rospy.Subscriber("/" + name + "/gripper/command", Float64, self.grip_2f_cb )

        else:
            rospy.Subscriber("/" + name + "/finger_1_joint_1_controller/command", Float64, self.grip_3f_1_cb )
            rospy.Subscriber("/" + name + "/finger_2_joint_1_controller/command", Float64, self.grip_3f_2_cb )
            rospy.Subscriber("/" + name + "/finger_middle_joint_1_controller/command", Float64, self.grip_3f_mid_cb )
            rospy.Subscriber("/" + name + "/palm_finger_1_joint_controller/command", Float64, self.grip_3f_palm_cb )
        

        # ------ Messages ------
        # Builds up the names and length of the message
        self.j_state = JointState()
        self.j_state.name = controlJoints + gripper_gripperJoints
        self.j_state.position = self.q + self.gripper
        self.j_state.velocity = [0,0,0,0,0,0] +  self.gripper
        self.j_state.effort = [0,0,0,0,0,0] + self.gripper

        # Robot cartesian pose
        self.pose = Pose()

        x = 0.4921                      # Cartesian origin
        y = 0.1334                       
        z = 0.4578

        roll = -1.57                   # Euler orientation origin
        pitch = -0.00079                   
        yaw = -3.14 

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
        cont = 0
        
        # Fills the joint states message
        # Robot joints values
        for i in self.ur5_joints_id:
            aux = p.getJointState(bodyUniqueId=self.ur5, 
                                jointIndex=i,
                                physicsClientId=self.client)

            self.j_state.position[cont] = aux[0]
            self.j_state.velocity[cont] = aux[1]

            cont = cont + 1

        # Gripper joint values
        for i in self.gripper_joints_id:
            aux = p.getJointState(bodyUniqueId=self.ur5, 
                                jointIndex=i,
                                physicsClientId=self.client)
            
            self.j_state.position[cont] = aux[0]
            self.j_state.velocity[cont] = aux[1]

            cont = cont + 1


        # Publish the joint states
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

    # Define mimic joints - 2f gripper
    def gripperControl2f(self):

        mimic_parent_name = 'finger_joint'
        mimic_children_names = {'left_inner_finger_joint': 1,
                                'left_inner_knuckle_joint': -1,
                                'right_outer_knuckle_joint': -1,
                                'right_inner_finger_joint': 1,  
                                'right_inner_knuckle_joint': -1}

        self.setup_mimic_joints_2f(mimic_parent_name, mimic_children_names)

    # Define mimic joints - 3f gripper
    def gripperControl3f(self, n):
        # Establish parent and children names for the mimic configuration
        mimic_parent_name = 'finger_' + n + '_joint_1'
        mimic_children_names = {'finger_' + n + '_joint_2': 1,
                                'finger_' + n + '_joint_3': 1}

        self.setup_mimic_joints_3f(mimic_parent_name, mimic_children_names)
      
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
    

    # Gripper callbacks: assigns the commands to each gripper joints
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

    def _joint_1_cb(self, data):
        p.setJointMotorControl2(bodyIndex=self.ur5, 
                                jointIndex=self.ur5_joints_id[0], 
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=data.data,
                                physicsClientId=self.client)

    def _joint_2_cb(self, data):
        p.setJointMotorControl2(bodyIndex=self.ur5, 
                                jointIndex=self.ur5_joints_id[1], 
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=data.data,
                                physicsClientId=self.client)
        
    def _joint_3_cb(self, data):
        p.setJointMotorControl2(bodyIndex=self.ur5, 
                                jointIndex=self.ur5_joints_id[2], 
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=data.data,
                                physicsClientId=self.client)
    
    def _joint_4_cb(self, data):
        p.setJointMotorControl2(bodyIndex=self.ur5, 
                                jointIndex=self.ur5_joints_id[3], 
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=data.data,
                                physicsClientId=self.client)
        
    def _joint_5_cb(self, data):
        p.setJointMotorControl2(bodyIndex=self.ur5, 
                                jointIndex=self.ur5_joints_id[4], 
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=data.data,
                                physicsClientId=self.client)
        
    def _joint_6_cb(self, data):
        p.setJointMotorControl2(bodyIndex=self.ur5, 
                                jointIndex=self.ur5_joints_id[5], 
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=data.data,
                                physicsClientId=self.client)