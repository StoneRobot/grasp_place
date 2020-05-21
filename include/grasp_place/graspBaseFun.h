#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8MultiArray.h>


#include "rubik_cube_solve/rubik_cube_solve_cmd.h"
#include "rubik_cube_solve/end_effector_motion.h"
#include "rubik_cube_solve/recordPoseStamped.h"
#include "hirop_msgs/openGripper.h"
#include "hirop_msgs/closeGripper.h"
#include "cubeParse/TakePhoto.h"
#include "cubeParse/SolveCube.h"
#include "std_msgs/Bool.h"

#include "hirop_msgs/RemoveObject.h"
#include "hirop_msgs/ShowObject.h"
#include "hirop_msgs/listActuator.h"
#include "hirop_msgs/listGenerator.h"
#include "hirop_msgs/SetGenActuator.h"
#include "hirop_msgs/ObjectArray.h"
#include "hirop_msgs/detection.h"
#include "grasp_place/rb_ArrayAndBool.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "yaml-cpp/yaml.h"

struct graspBaseFun
{
    int pickRobot;
    // 0 牛奶
    // 1 可乐
    int pickObject;
    // 0 从货架得到桌子
    // 1 从桌子到货架
    int pick_mode;
};


class GraspPlace
{
    GraspPlace(ros::NodeHandle n, moveit::planning_interface::MoveGroupInterface& group0, moveit::planning_interface::MoveGroupInterface& group1);

    void rmObject();
    void showObject(geometry_msgs::Pose pose);
    bool setGenActuator();
    bool transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id);
    void robotMoveCartesianUnit2(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z);
    moveit::planning_interface::MoveGroupInterface& getMoveGroup(int num);
    bool closeGripper(moveit::planning_interface::MoveGroupInterface& move_group);
    bool openGripper(moveit::planning_interface::MoveGroupInterface& move_group);

    void PickPlace(int robotNum, geometry_msgs::PoseStamped& pose, bool isPick, int pre_grasp_approach[], int post_grasp_retreat[]);
    moveit::planning_interface::MoveItErrorCode setAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                        geometry_msgs::PoseStamped& poseStamped);
    moveit::planning_interface::MoveItErrorCode moveGroupPlanAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                moveit::planning_interface::MoveGroupInterface::Plan& my_plan);

    moveit::planning_interface::MoveItErrorCode loop_move(moveit::planning_interface::MoveGroupInterface& move_group);
    void objectCallback(const hirop_msgs::ObjectArray::ConstPtr& msg);
    void getPickDataCallBack(grasp_place::rb_ArrayAndBool::Request& req, grasp_place::rb_ArrayAndBool::Response& rep);
    void detectionObject(int objectNum);

private:
    ros::NodeHandle nh;
    moveit::planning_interface::MoveGroupInterface& move_group0;
    moveit::planning_interface::MoveGroupInterface& move_group1;
    ros::ServiceClient openGripper_client0;
    ros::ServiceClient closeGripper_client0;
    ros::ServiceClient openGripper_client1;
    ros::ServiceClient closeGripper_client1;
    ros::ServiceClient list_generator_client;
    ros::ServiceClient set_gen_actuator_client;
    ros::ServiceClient list_actuator_client;
    ros::ServiceClient show_object_client;
    ros::ServiceClient remove_object_client;
    ros::ServiceServer getPickData;
    ros::ServiceClient detection_client;
    ros::Subscriber pose_sub;
    ros::Publisher Object_pub;
    graspBaseFun pickData;
    const double prepare_some_distance = 0.08;
    const std::vector<std::string> pickObjectName={"milk_shelf"};
    std::vector<geo>
};
