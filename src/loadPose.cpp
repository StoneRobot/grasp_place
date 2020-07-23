#include "grasp_place/loadPose.h"

recordLoadPose::recordLoadPose()
{
}


void recordLoadPose::loadRobotPose(geometry_msgs::PoseStamped& pose, std::string path)
{
    YAML::Node doc;
    doc = YAML::LoadFile(path);
    addData(pose, doc);
}

void recordLoadPose::addData(geometry_msgs::PoseStamped& pose, YAML::Node node)
{
    pose.header.frame_id = node["header"]["frame_id"].as<std::string>();
    pose.pose.position.x = node["pose"]["position"]["x"].as<double>();
    pose.pose.position.y = node["pose"]["position"]["y"].as<double>();
    pose.pose.position.z = node["pose"]["position"]["z"].as<double>();
    pose.pose.orientation.x = node["pose"]["orientation"]["x"].as<double>();
    pose.pose.orientation.y = node["pose"]["orientation"]["y"].as<double>();
    pose.pose.orientation.z = node["pose"]["orientation"]["z"].as<double>();
    pose.pose.orientation.w = node["pose"]["orientation"]["w"].as<double>();
}

void recordLoadPose::recordRobotPose(geometry_msgs::PoseStamped& pose, std::string name)
{
    std::ofstream fout(name, std::ios::out);
    YAML::Node config;
    config["header"]["frame_id"] = pose.header.frame_id;
    config["pose"]["position"]["x"] = pose.pose.position.x;
    config["pose"]["position"]["y"] = pose.pose.position.y;
    config["pose"]["position"]["z"] = pose.pose.position.z;
    config["pose"]["orientation"]["x"] = pose.pose.orientation.x;
    config["pose"]["orientation"]["y"] = pose.pose.orientation.y;
    config["pose"]["orientation"]["z"] = pose.pose.orientation.z;
    config["pose"]["orientation"]["w"] = pose.pose.orientation.w;
    fout << config;
    ROS_INFO_STREAM("write over " << name.c_str());
    fout.close();
}