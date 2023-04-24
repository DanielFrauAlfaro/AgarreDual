#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <iostream>
#include <algorithm>

std_msgs::Float64 cmd;

void cmd_cb_2f(const std_msgs::Float64::ConstPtr& data)
{
    cmd.data = data->data;
}

int main(int argc, char **argv)
{
    std::string name = argv[1];
    std::string model = argv[2];

    ros::init(argc, argv, "main_gripper_controller");
    ros::NodeHandle n;

    ros::Rate r(10);

    ros::Subscriber sub_cmd = n.subscribe("grip_cmd", 1000, &cmd_cb_2f);

    auto joint_pub = n.advertise<std_msgs::Float64>("gripper/command", 10);
        
  
    while(ros::ok())
    {
        joint_pub.publish(cmd);
        
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}