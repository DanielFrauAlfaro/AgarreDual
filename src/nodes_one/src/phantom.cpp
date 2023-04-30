#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Joy.h"

#include <iostream>
#include <chrono>
#include <cmath>

// Messages
geometry_msgs::Pose pose;               // Effector position
geometry_msgs::WrenchStamped wrench;    // Phantom wrench
std_msgs::Float64 msg_grip;             // Gripper position

// Joint position of the gripper
float grip_pos = 0.0;

// Movements scalation
int scale_x = 2;                        // Cartesian movement scalation
int scale_y = 2;
float scale_z = 3.5;

int scale_roll = 15;                    // Euler orientation movement scalation
int scale_pitch = 15;
int scale_yaw = 15;

float scale_grip = 0;                   // Gripper scalation
float scale_grip_3f = 1342.1;
float scale_grip_2f = 5.5;

// Origin
float or_x = 0.4921;                    // Cartesian origin
float or_y = 0.1334;
float or_z = 0.4578;

float or_roll = -1.57;                  // Euler orientation origin
float or_pitch = -0.00079;
float or_yaw = -3.14;

float or_3f = 0.0;                      // 3f gripper origin

// Previous positions and orientation in the main loop
float prev_x = or_x, prev_y = or_y, prev_z = or_z;
float prev_roll = or_roll, prev_pitch = or_pitch, prev_yaw = or_yaw;

// Previous position of the gripper
float prev_grip_pos = 0.0;


// Modes and Flags
bool change = true;                     // Flag to indicate a mode change
std_msgs::Int32 msg_change;             // Movement state
int state = 0;
int state_3f = 0;                       // 3f gripper movement state 


// Actual and previous position and orientation of the Phantom
geometry_msgs::Pose prev_pose_phantom;
geometry_msgs::Pose act_pose_phantom;


// Force feedback gains (PD)
const int K = 40;                       // Positional gains
const float KD = 0.2;

const int K_or = 9;                     // Orientational gains
const float KD_or = 0.06;

const int K_grip_2f = 10.0;             // 2f gripper joints gains
const float KD_grip_2f = 0.05;

const float K_grip_3f = 0.1;            // 3f gripper joints gains
const float KD_grip_3f = 0.003;

const int Ke = 70;                      // Mode change gain                  
const float Kde = 0.2;

// Detection umbral
const float limit = 0.005;

// PI
const float pi = 3.1416;


// Names
std::string name = "ur5_2";
std::string grip = "3f";


// Interval and times
auto prev = std::chrono::high_resolution_clock::now();;
const float interval = 0.07 * 1000;

// Phantom cartesian position callback
/*
The robot reference system is different than the one of the Phantom --> 
  - x_robot = z_phantom
  - y_robot = x_phantom
  - z_robot = y_phantom
  
Three variables: 
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
*/
void cb(const geometry_msgs::PoseStamped::ConstPtr& data)
{
    auto now = std::chrono::high_resolution_clock::now();
    auto mseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - prev).count();

    if(mseconds > interval)
    {
        prev = std::chrono::high_resolution_clock::now();
        
        // 1. --
        act_pose_phantom.position = data->pose.position;
        
        if(!change)
        {
            // 2.1. --
            if(state == 0)
            {
                pose.position.x = data->pose.position.z * scale_x + or_x;
                pose.position.y = data->pose.position.x * scale_y + or_y;
                pose.position.z = data->pose.position.y * scale_z + or_z;
                
                pose.orientation.x = prev_roll;
                pose.orientation.y = prev_pitch;
                pose.orientation.z = prev_yaw;
                
                prev_pose_phantom.position = data->pose.position;
            }

            // 2.2. --
            else if(state == 1)
            {
                pose.position.x = prev_x;
                pose.position.y = prev_y;
                pose.position.z = prev_z;

                pose.orientation.x = data->pose.position.z * scale_roll + or_roll;
                pose.orientation.y = data->pose.position.x * scale_pitch + or_pitch;
                pose.orientation.z = data->pose.position.y * scale_yaw + or_yaw;
                
                prev_pose_phantom.orientation.x = data->pose.position.x;
                prev_pose_phantom.orientation.y = data->pose.position.y;
                prev_pose_phantom.orientation.z = data->pose.position.z;
            }

            // 2.3. --
            else if(state == 2 && data->pose.position.y >= 0.0)
            {
                grip_pos = data->pose.position.y * scale_grip;
            }
        }
    }
}

// Button 1 callback
void cb_bt1(const sensor_msgs::Joy::ConstPtr& data)
{
    if(data->buttons[0] == 1)
    {
        state++;                       // Advances the state
        change = true;                 // A change happened
        state_3f = 0;
    }

    // If the number exceeds the total state, returns to state 0
    if(state > 2)           
        state = 0;
    
    // Publishes the change
    msg_change.data = -(state+1);
}

// Button 2 callback
void cb_bt2(const sensor_msgs::Joy::ConstPtr& data)
{
    if(data->buttons[0] == 1)
    {
        state--;                        // Reduces the state
        change = true;                  // A change happened
    }

    // If the number exceeds the total state, returns to state 0
    if(state < 0)           
        state = 2;
    
    // Publishes the change
    msg_change.data = -(state+1);
}

// Robot cartesian position feedback
void cart_pos(const geometry_msgs::Pose::ConstPtr& data)
{   
    // Gets the position in every mode except in orientation
    if(state != 1)
    {
        prev_x = data->position.x;
        prev_y = data->position.y;
        prev_z = data->position.z;
    }

    // Gets the orientation in every mode except in position
    if(state != 0)
    {
        prev_roll = data->orientation.x;
        prev_pitch = data->orientation.y;
        prev_yaw = data->orientation.z;
    }
}

// Gripper position callback
void grip_pos_cb(const std_msgs::Float32::ConstPtr& data)
{
    prev_grip_pos = data->data;
}

// ----- Main -----
int main(int argc, char **argv)
{
    //Initializes the phantom positions
    prev_pose_phantom.position.x = 0.0;
    prev_pose_phantom.position.y = 0.0;
    prev_pose_phantom.position.z = 0.0;
    prev_pose_phantom.orientation.x = 0.0;
    prev_pose_phantom.orientation.y = 0.0;
    prev_pose_phantom.orientation.z = 0.0;

    act_pose_phantom.position.x = -1.0;
    act_pose_phantom.position.y = -1.0;
    act_pose_phantom.position.z = -1.0;
    act_pose_phantom.orientation.x = -1.0;
    act_pose_phantom.orientation.y = -1.0;
    act_pose_phantom.orientation.z = -1.0;

    // Initializes the robot name and gripper types
    name = argv[1];
    grip = argv[2];

    // Adjust gripper scales
    if(grip == "3f")
    {
        scale_grip = scale_grip_3f;
    }

    else if(grip == "2f_140")
    {
       scale_grip = scale_grip_2f;
    }

    // Node
    ros::init(argc, argv, name + "_phantom");
    ros::NodeHandle n;

    // ------ Publishers ------
    // Cartesian position command
    ros::Publisher pub = n.advertise<geometry_msgs::Pose>("pose", 10);
    
    // Phantom cartesian force
    ros::Publisher pub_f = n.advertise<geometry_msgs::WrenchStamped>("servo_cf", 10);
    
    // System changes and state
    ros::Publisher pub_change = n.advertise<std_msgs::Int32>("change", 10);
    
    // Position command for the grippers
    ros::Publisher pub_grip = n.advertise<std_msgs::Float64>("grip_cmd", 10);


    // ------ Subscribers ------
    // Phantom cartesian position
    ros::Subscriber sub_phantom = n.subscribe("measured_cp", 1000, cb);
    
    // Phantom buttons
    ros::Subscriber sub_bt1 = n.subscribe("button1", 1000, cb_bt1);
    ros::Subscriber sub_bt2 = n.subscribe("button2", 1000, cb_bt2);
    
    // Gripper position feedcback
    ros::Subscriber sub_cart_pos = n.subscribe("cart_pos", 1000, cart_pos);
    
    // Gripper publisher
    ros::Subscriber sub_grip_state = n.subscribe("grip_state", 1000, grip_pos_cb);

    // Timer
    auto t = std::chrono::high_resolution_clock::now();

    // Rate
    ros::Rate r(18);

    // Initializes errors
    float ex = 0.0;
    float ey = 0.0;
    float ez = 0.0;

    float ex0 = 0.0;
    float ey0 = 0.0;
    float ez0 = 0.0;
    
    msg_change.data = -1;
    /* Control loop: sends catesian commands to the robot controller and applies forces
        to the manipulator according to the error between the robot position and 
        the Phantom one */
    while(ros::ok())
    {   
        // If there has been a change, computes the error 
        //    the previous position of that mode
        if(change)
        {
            pub_change.publish(msg_change);

            if(state == 0)
            {
                
                ex = (prev_pose_phantom.position.x - act_pose_phantom.position.x);
                ey = (prev_pose_phantom.position.y - act_pose_phantom.position.y);
                ez = (prev_pose_phantom.position.z - act_pose_phantom.position.z);
            }

            else if(state == 1)
            {
                ex = (prev_pose_phantom.orientation.x - act_pose_phantom.position.x);
                ey = (prev_pose_phantom.orientation.y - act_pose_phantom.position.y);
                ez = (prev_pose_phantom.orientation.z - act_pose_phantom.position.z);
            }

            else
            {
                ex = (0.0 - act_pose_phantom.position.x);
                ey = (prev_grip_pos / scale_grip - act_pose_phantom.position.y);
                ez = (0.0 - act_pose_phantom.position.z);
            }

            // If the Phantom end effector is within the previous position, 
            //   ends the mode changing state
            if(abs(ex) < limit && abs(ey) < limit && abs(ez) < limit)
            {
                change = false;
                msg_change.data = state + 1;
                pub_change.publish(msg_change);
            }
        }

        // If there has not been a mode changing, the Phantom sends position to the 
        //     robot and applies forces to the operator to restrict his / her movements and
        //     minimize errors. Force is computed by the error 
        else
        {
            //Publishes the robot position
            pub.publish(pose);

            // Publishes the gripper position
            msg_grip.data = grip_pos;
            pub_grip.publish(msg_grip);


            if(state == 0)
            {
                ex = (prev_y - pose.position.y);
                ey = (prev_z - pose.position.z);
                ez = (prev_x - pose.position.x);
            }
                
            else if(state == 1)
            {
                ex = (prev_pitch - pose.orientation.y);

                // Corrects the orientation so it remains inside the [-pi,pi] interval
                if(prev_yaw < 0.0)
                {
                    prev_yaw = 2*pi + prev_yaw;
                }

                if(pose.orientation.z < 0.0)
                {
                    pose.orientation.z = 2*pi + pose.orientation.z;
                }

                ey = (prev_yaw - pose.orientation.z);
                ez = (prev_roll - pose.orientation.x);
            }

            // On the gripper mode, the Phantom can not go to negative positions
            else
            {
                ex = (0 - act_pose_phantom.position.x) * 10;
                ez = (0 - act_pose_phantom.position.z) * 10;
                ey0 = (0 - act_pose_phantom.position.y) * 10;

                ey = (prev_grip_pos - grip_pos); 
                
                // If some coordinate is negative, the error is the one between the 
                //     origin and the actual position
                if(act_pose_phantom.position.y < 0.0)
                {
                    ey = ey0;

                    if(grip == "3f")
                    {
                        ey = ey0 * 80;
                    }
                }
            }
        }
        
        // Depending on the mode, applies a specific gain
        float k = 0;
        float kd = 0;

        if(change)
        {
            k = Ke;
            kd = Kde;
        }

        else if(state == 0)
        {
            k = K;
            kd = KD;
        }
        
        else if(state == 1)
        {
            k = K_or;
            kd = KD_or;
        }
        
        else
        {
            if(grip == "2f_140")
            {
                k = K_grip_2f;
                kd = KD_grip_2f;
            }

            else if(grip == "3f")
            {
                k = K_grip_3f;
                kd = KD_grip_3f * 0;

                ez = ez * 80;
                ex = ex * 80;
            }
        }


        // Computes the forces
        auto now = std::chrono::high_resolution_clock::now();
        auto mseconds = float(std::chrono::duration_cast<std::chrono::milliseconds>(now - t).count()) / 1000.0;

        wrench.wrench.force.x = ex * k - ex / float(mseconds) * kd;
        wrench.wrench.force.y = ey * k - ey / float(mseconds) * kd;
        wrench.wrench.force.z = ez * k - ez / float(mseconds) * kd;

        // Upwards constant force if there is not any change
        if(!change && state == 0)
        {
            wrench.wrench.force.y = wrench.wrench.force.y + 0.9;
        }

        // Wrench publisher
        pub_f.publish(wrench);

        // Updates time
        t = std::chrono::high_resolution_clock::now();

        // Spin and sleep
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
