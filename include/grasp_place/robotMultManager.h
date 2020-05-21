#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

class RobotMultManager
{
private:
    
public:
    RobotMultManager(ros::NodeHandle nodehandle);
    ~RobotMultManager();
};

RobotMultManager::RobotMultManager(ros::NodeHandle nodehandle)
{
}

RobotMultManager::~RobotMultManager()
{
}
