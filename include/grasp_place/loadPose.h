#pragma once
#include "yaml-cpp/yaml.h"
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>

class recordLoadPose
{
private:
    void addData(geometry_msgs::PoseStamped& pose, YAML::Node node);
    // ros::NodeHandle nh;
public:
    recordLoadPose();
    void loadRobotPose(geometry_msgs::PoseStamped& pose, std::string path);
    void recordRobotPose(geometry_msgs::PoseStamped& pose, std::string FileName);
};

