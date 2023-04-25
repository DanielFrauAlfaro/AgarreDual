#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <iostream>
#include <algorithm>

// Command message
std_msgs::Float64 cmd;

// Command callback
void cmd_cb_2f(const std_msgs::Float64::ConstPtr& data)
{
    cmd.data = data->data;
}


// ---- Main ----
int main(int argc, char **argv)
{
    // Names
    std::string name = argv[1];
    std::string model = argv[2];

    // Node
    ros::init(argc, argv, "main_gripper_controller");
    ros::NodeHandle n;

    // Rate
    ros::Rate r(5);

    // ----- Publishers -----
    // Publisher of the gripper joint values
    auto joint_pub = n.advertise<std_msgs::Float64>("gripper/command", 10);

    // ----- Subscribers -----
    ros::Subscriber sub_cmd = n.subscribe("grip_cmd", 1000, &cmd_cb_2f);
        
    
    // Infinite loop
    while(ros::ok())
    {
        // Publish command
        joint_pub.publish(cmd);
        
        // Spin and sleep
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}