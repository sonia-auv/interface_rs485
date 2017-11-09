#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "interface_rs485");

    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    interface_rs485::InterfaceRs485Node interface_node{nh};
    interface_node.Spin();
}
