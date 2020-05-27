#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "hirop_msgs/ObjectArray.h"

ros::Publisher planning_scene_diff_publisher;

inline double angle2rad(double angle)
{
    return angle/180*3.1415;
}

moveit_msgs::CollisionObject addCollisionObjects(double sx, double sy, double sz,\
double px, double py, double pz, \
double ox, double oy, double oz, \
std::string frame_id, std::string id)
{

    moveit_msgs::CollisionObject collision_objects;

    collision_objects.id = id;
    collision_objects.header.frame_id = frame_id;

    collision_objects.primitives.resize(1);
    collision_objects.primitives[0].type = collision_objects.primitives[0].BOX;
    collision_objects.primitives[0].dimensions.resize(3);
    collision_objects.primitives[0].dimensions[0] = sx;
    collision_objects.primitives[0].dimensions[1] = sy;
    collision_objects.primitives[0].dimensions[2] = sz;

    collision_objects.primitive_poses.resize(1);
    collision_objects.primitive_poses[0].position.x = px;
    collision_objects.primitive_poses[0].position.y = py;
    collision_objects.primitive_poses[0].position.z = pz;

    tf2::Quaternion orientation;
    orientation.setRPY(angle2rad(ox), angle2rad(oy), angle2rad(oz));
    collision_objects.primitive_poses[0].orientation = tf2::toMsg(orientation);

    collision_objects.operation = collision_objects.ADD;

    return collision_objects;
    // moveit_msgs::PlanningScene p;
    // p.world.collision_objects.push_back(collision_objects[0]);
    // p.is_diff = true;
    // p.robot_state.is_diff = true;
    // planning_scene_diff_publisher.publish(p);
    
}

void remove()
{
    moveit_msgs::PlanningScene p;
    std::vector<std::string> objectName={"box1", "box2", "box3", "box4", "box5"};
    for(auto it: objectName)
    {
        moveit_msgs::CollisionObject remove_object;
        remove_object.operation = remove_object.REMOVE;
        remove_object.header.frame_id = "world";
        remove_object.id = it;
        p.world.collision_objects.push_back(remove_object);
    }
    planning_scene_diff_publisher.publish(p);
}

void addShelf(double x)
{
    moveit_msgs::PlanningScene p;
    p.world.collision_objects.resize(5);
    p.world.collision_objects[0] = addCollisionObjects(0.2, 1.2, 0.02, x, 0.5, 1.84, 0, 0, 0, "world", "box1");
    p.world.collision_objects[1] = addCollisionObjects(0.2, 1.2, 0.02, x, 0.5, 1.57, 0, 0, 0, "world", "box2");
    p.world.collision_objects[2] = addCollisionObjects(0.2, 1.2, 0.02, x, 0.5, 1.31, 0, 0, 0, "world", "box3");
    p.world.collision_objects[3] = addCollisionObjects(0.2, 0.01, 1.84, x, 1.1, 0.9, 0, 0, 0, "world", "box4");
    p.world.collision_objects[4] = addCollisionObjects(0.2, 0.01, 1.84, x, -0.1, 0.9, 0, 0, 0, "world", "box5");
    planning_scene_diff_publisher.publish(p);
    // // 2 2 0.01
    // // 0 0.5 1.05 
    // // 0 0 0 wolrd floor
    // std::string cmd0 = "rosrun rubik_cube_solve add 0.2 1.2 0.02 ";
    // std::string cmd1 = " 0.6 1.84 0 0 0 world top";
    // std::string cmd2 = " 0.6 1.57 0 0 0 world shelftop";
    // std::string cmd3 = " 0.6 1.31 0 0 0 world shelfbottom";
    // std::string cmd4 = cmd0 + std::to_string(x) + cmd1;
    // std::string cmd5 = cmd0 + std::to_string(x) + cmd2;
    // std::string cmd6 = cmd0 + std::to_string(x) + cmd3;
    // system(cmd4.c_str());
    // ROS_INFO_STREAM(1);
    // system(cmd5.c_str());
    // ROS_INFO_STREAM(2);
    // system(cmd5.c_str());
    // ROS_INFO_STREAM(3);
    // ///////////////////////////////////
    // std::string cmd7 = "rosrun rubik_cube_solve add 0.2 0.01 1.84 ";
    // std::string cmd8 = " 1.1 0.9 0 0 0 world r";
    // std::string cmd9 = " -0.1 0.9 0 0 0 world l";
    // std::string cmd10 = cmd7 + std::to_string(x) + cmd8;
    // std::string cmd11 = cmd7 + std::to_string(x) + cmd9;
    // system(cmd10.c_str());
    // ROS_INFO_STREAM(4);
    // system(cmd11.c_str());
    // ROS_INFO_STREAM(5);
}

bool robotMoveCartesian(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z)
{
    std::vector<double>joint = group.getCurrentJointValues();
    moveit_msgs::RobotState r;
    r.joint_state.position = joint;
    group.setStartState(r);
    group.setStartStateToCurrentState();
    geometry_msgs::PoseStamped temp_pose = group.getCurrentPose();

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    temp_pose.pose.position.x +=x;
    temp_pose.pose.position.y +=y;
    temp_pose.pose.position.z +=z;
    geometry_msgs::Pose target_pose = temp_pose.pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    int cnt = 0;
    bool isSuccess;
    do
    {
        isSuccess = (group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        cnt ++;
    }
    while (isSuccess == false && cnt < 100);
    if(isSuccess)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        group.execute(plan);
        return true;
    }
    else
    {
        return false;
    }
}

bool planAndGo(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
{
    int cnt = 0;
    bool isSuccess;
    do
    {
        isSuccess = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        cnt ++;
    }
    while (isSuccess == false && cnt < 5);
    if(isSuccess)
    {
        move_group.move();
        return true;
    }
    else
    {
        return false;
    }
}

bool moveGroupGo(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::PoseStamped targetPose)
{
    const double x = 0.07;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    targetPose.pose.position.x += x;
    move_group.setPoseTarget(targetPose);
    if(planAndGo(move_group, my_plan))
        if(robotMoveCartesian(move_group, -x, 0, 0))
            if(robotMoveCartesian(move_group, x, 0, 0))
                return true;
    return false;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grasp_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group0("arm0");
    moveit::planning_interface::MoveGroupInterface move_group1("arm1");
    planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
     {
        ros::Duration(0.5).sleep();
    }
    ros::Publisher objectPub = nh.advertise<hirop_msgs::ObjectArray>("object_array", 1);
    double xDistanceBegin = 0.6;
    // -0.6 - 0.05 * 7 = -0.95
    double xDistance = -0.60;
    // 上下层抓取的位置
    const double zShelfTop = 1.63;
    const double zShelfBottom = 1.37;
    double zShelf;
    geometry_msgs::PoseStamped targetPose;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 3.14);
    targetPose.header.frame_id = "world";
    targetPose.pose.orientation = tf2::toMsg(orientation);
    // 存储实验数据的文档
    std::ofstream destFile("recored.txt", std::ios::out);
    if(!destFile)
    {
        ROS_INFO_STREAM("open file faild");
        return -1;
    }
    
    // X 轴的距离
    for(std::size_t i = 0; i < 7; ++i)
    {
        destFile << "x distance: " << xDistance << std::endl;
        addShelf(xDistance);
        for(std::size_t robot=0; robot<2; ++robot)
        {
            destFile << "robot: " << robot << std::endl;
            // 上下层
            for(std::size_t j=0; j<2; ++j)
            {
                int successPoint[2][16] = {0};
                std::string shelf;
                // 右边的机器人从0.2m开始(Y 轴)
                double yDistance = robot * 0.2;
                // double yDistance = 0.4;
                switch (j)
                {
                    case 0:
                        shelf = "top";
                        zShelf = zShelfTop;
                        break;
                    default:
                        shelf = "bottom";
                        zShelf = zShelfBottom;
                        break;
                }
                double successRate = 0;
                bool isSuccess;
                int cnt = 0;
                // 由货架从左向右, 从0开始, 0.05间距, 1米结束
                for(std::size_t k=0; k<16; ++k)
                {
                    targetPose.pose.position.x = xDistance;
                    targetPose.pose.position.y = yDistance;
                    targetPose.pose.position.z = zShelf;
                    ROS_INFO("robot: %d, shelf: %d, k: %d", robot, j, k);
                    ROS_INFO_STREAM(targetPose);
                    // ROS_INFO_STREAM(robot << " " << targetPose);
                    switch (robot)
                    {
                        case 0:
                            isSuccess = moveGroupGo(move_group0, targetPose);
                            break;
                        case 1:
                            isSuccess = moveGroupGo(move_group1, targetPose);
                            break;
                    }
                    if(isSuccess)
                    {
                        cnt ++;
                        successPoint[j][k] = 1;
                    }
                    yDistance += 0.05;
                }
                successRate = static_cast<double>(cnt)/16.0;
                destFile << shelf.c_str() << " " << successRate << std::endl;
            }
            if(robot==0)
            {
                move_group0.setNamedTarget("home0");
                move_group0.move();
            }
            else
            {
                move_group1.setNamedTarget("home1");
                move_group1.move();
            }
            destFile << "......" <<std::endl;
        }
        xDistance -= 0.05;
        destFile << "--------------------------------" <<std::endl;
        remove();
    }
    destFile.close();
    return 0;
}