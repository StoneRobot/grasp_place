#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <stdlib.h>

void addShelf(double x)
{
    // 2 2 0.01
    // 0 0.5 1.05 
    // 0 0 0 wolrd floor
    std::string cmd0 = "rosrun rubik_cube_solve add 0.2 1.2 0.02 ";
    std::string cmd1 = " 0.6 1.84 0 0 0 world top";
    std::string cmd2 = " 0.6 1.57 0 0 0 world shelftop";
    std::string cmd3 = " 0.6 1.31 0 0 0 world shelfbottom";
    std::string cmd4 = cmd0 + std::to_string(x) + cmd1;
    std::string cmd5 = cmd0 + std::to_string(x) + cmd2;
    std::string cmd6 = cmd0 + std::to_string(x) + cmd3;
    system(cmd4.c_str());
    system(cmd5.c_str());
    system(cmd5.c_str());
    ///////////////////////////////////
    std::string cmd7 = "rosrun rubik_cube_solve add 0.2 0.01 1.84 ";
    std::string cmd8 = " 1.1 0.9 0 0 0 world r";
    std::string cmd9 = " -0.1 0.9 0 0 0 world l";
    std::string cmd10 = cmd7 + std::to_string(x) + cmd8;
    std::string cmd11 = cmd7 + std::to_string(x) + cmd9;
    system(cmd10.c_str());
    system(cmd11.c_str());
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grasp_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group0("arm0");
    moveit::planning_interface::MoveGroupInterface move_group1("arm1");
    double xDistanceBegin = 0.6;
    // 
    for (std::size_t i = 0; i < 7; i++)
    {
        for(std::size_t j=0; )
    }
    return 0;
}