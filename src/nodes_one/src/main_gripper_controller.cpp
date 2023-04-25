#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <cmath>

float pi = 3.1416;

// Gripper constants
const int th[4] = {110, 140, 240, 255};
const float max_rot[3] = {1.22, float(pi * 0.5), -0.0523};
const float min_rot[3] = {0.0495, 0.0, -0.96};

const float m1 = 1.22 / 140.0;
const float m2 = pi / 2.0 / 100.0;

// Command Message 
std_msgs::Float64MultiArray msg1;


// Callback for the command topic
void cmd_cb_3f(const std_msgs::Float64::ConstPtr& data)
{
    float g = data->data;

    // Applies the 3f Gripper state machine
    if(g >= 0.0 && g <= th[1])
    {
        msg1.data[0] = m1 * g;
        msg1.data[1] = min_rot[1];
        msg1.data[2] = std::max(-m1 * g, min_rot[2]);

        msg1.data[3] = m1 * g;
        msg1.data[4] = min_rot[1];
        msg1.data[5] = std::max(-m1 * g, min_rot[2]);

        msg1.data[6] = m1 * g;
        msg1.data[7] = min_rot[1];
        msg1.data[8] = std::max(-m1 * g, min_rot[2]);
        
    }

    else if(g <= th[2])
    {
        msg1.data[0] = max_rot[0];
        msg1.data[1] = m2 * (g - th[1]);
        msg1.data[2] = min_rot[2];

        msg1.data[3] = max_rot[0];
        msg1.data[4] = m2 * (g - th[1]);
        msg1.data[5] = min_rot[2];

        msg1.data[6] = max_rot[0];
        msg1.data[7] = m2 * (g - th[1]);
        msg1.data[8] = min_rot[2];
    }

    else
    {
        msg1.data[0] = max_rot[0];
        msg1.data[1] = min_rot[1];
        msg1.data[2] = min_rot[2];

        msg1.data[3] = max_rot[0];
        msg1.data[4] = min_rot[1];
        msg1.data[5] = min_rot[2];

        msg1.data[6] = max_rot[0];
        msg1.data[7] = min_rot[1];
        msg1.data[8] = min_rot[2];
    }
}


// ---- Main ----
int main(int argc, char **argv)
{
    // Initializes the command message
    msg1.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // Name and gripper model from arguments
    std::string name = argv[1];

    // Node
    ros::init(argc, argv, "main_gripper_controller");
    ros::NodeHandle n;

    // Rate
    ros::Rate r(5);

    // ----- Publishers -----
    // Joint values publisher
    auto joint_pub1 = n.advertise<std_msgs::Float64MultiArray>("gripper_controller/command", 10);

    // ----- Subscribers -----
    // Command subscribers
    ros::Subscriber sub_cmd = n.subscribe("grip_cmd", 1000, &cmd_cb_3f);


    // Infinite loop
    while(ros::ok())
    {
        // Publishes the message
        joint_pub1.publish(msg1);

        // Spin and sleep
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}