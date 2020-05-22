#include "grasp_place/graspBaseFun.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grasp_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group0("arm0");
    moveit::planning_interface::MoveGroupInterface move_group1("arm1");
    GraspPlace g(nh, move_group0, move_group1);
    ros::waitForShutdown();
    return 0;
}
