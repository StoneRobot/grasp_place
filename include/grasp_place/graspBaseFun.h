#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <grasp_place/loadPose.h>

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
#include "rb_msgAndSrv/rb_ArrayAndBool.h"

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
    int pickMode;
};


class GraspPlace
{
public:
    GraspPlace(ros::NodeHandle nodehandle, moveit::planning_interface::MoveGroupInterface& group0, moveit::planning_interface::MoveGroupInterface& group1);

    void showObject(geometry_msgs::Pose pose);
    bool transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id);
    bool robotMoveCartesianUnit2(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z);
    moveit::planning_interface::MoveGroupInterface& getMoveGroup(int num);
    bool closeGripper(moveit::planning_interface::MoveGroupInterface& move_group);
    bool openGripper(moveit::planning_interface::MoveGroupInterface& move_group);

    bool PickPlace(int robotNum, geometry_msgs::PoseStamped& pose, bool isPick, int pre_grasp_approach[], int post_grasp_retreat[]);
    moveit::planning_interface::MoveItErrorCode setAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                        geometry_msgs::PoseStamped& poseStamped);
    moveit::planning_interface::MoveItErrorCode moveGroupPlanAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                moveit::planning_interface::MoveGroupInterface::Plan& my_plan);

    moveit::planning_interface::MoveItErrorCode loop_move(moveit::planning_interface::MoveGroupInterface& move_group);
    bool detectionObject(int objectNum, int robot);
    bool pick(geometry_msgs::PoseStamped pose);
    bool place(); 

    bool loadPose(std::string folder, std::vector<std::vector<std::string> > filePathParam, std::vector<std::vector<geometry_msgs::PoseStamped> >& pose);
    bool loadPickObjectName();


    void preprocessingPlacePose();
    void calibration(std::vector<std::vector<geometry_msgs::PoseStamped> > pose, std::vector<std::vector<std::string> > poseName, std::string folder, bool isNowHavePoseFile);
    void calibrationDetection(bool isNowHavePoseFile);
    void calibrationPlace(bool isNowHavePoseFile);
    void removeOrAddObject();
    // std::string showTF(geometry_msgs::PoseStamped pose);
    void stopMove();
    void backHome(int robotNum);
    void rmObject(std::string name);
    void stopAndBackHome(int robotNum);
    void setStartState(moveit::planning_interface::MoveGroupInterface& move_group);

    bool move(moveit::planning_interface::MoveGroupInterface& move_group, \
                                geometry_msgs::PoseStamped& poseStamped);
private:

    bool test(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::PoseStamped& poseStamped);
    /**
     * @brief 设置机器人摆放物品的姿态
     * @param RPY 
     * 0 货架
     * 1 左机器人
     * 2 右机器人
    */
    geometry_msgs::PoseStamped changeOrientation(geometry_msgs::PoseStamped& pose, int RPY);

    /**
     * @brief 
     * @param type 
     * 0 货架
     * 1 左机器人
     * 2 右机器人
    */
    void preprocessingPlacePose(geometry_msgs::PoseStamped& pose, int type);

    bool code2bool(moveit::planning_interface::MoveItErrorCode code);
    bool writePoseOnceFile(const std::string& name, const geometry_msgs::PoseStamped& pose);
    bool addData(geometry_msgs::PoseStamped& pose, YAML::Node node);
    bool recordPose(int robotNum, std::string name, bool isJointSpace, std::string folder);

    void objectCallBack(const hirop_msgs::ObjectArray::ConstPtr& msg);
    bool pickPlaceObject(geometry_msgs::PoseStamped pickPose);
    
    void sotpMoveCallback(const std_msgs::Bool::ConstPtr& msg);
    // 0 爲檢測, 1 爲放置, 2, 3同是, 只是之前沒有點位文件, 或不用點位文件.
    void calibrationCallBack(const std_msgs::Int8::ConstPtr& msg);
    void backHomeCallback(const std_msgs::Int8::ConstPtr& msg);
    
    bool getPickDataCallBack(rb_msgAndSrv::rb_ArrayAndBool::Request& req, rb_msgAndSrv::rb_ArrayAndBool::Response& rep);
    bool recordPoseCallBack(rubik_cube_solve::recordPoseStamped::Request& req, rubik_cube_solve::recordPoseStamped::Response& rep);
 
    ros::NodeHandle nh;
    moveit::planning_interface::MoveGroupInterface& move_group0;
    moveit::planning_interface::MoveGroupInterface& move_group1;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::ServiceClient openGripper_client0;
    ros::ServiceClient closeGripper_client0;
    ros::ServiceClient openGripper_client1;
    ros::ServiceClient closeGripper_client1;
    ros::ServiceClient detection_client;
    ros::ServiceClient detection_client_right;

    ros::ServiceServer getPickData;
    ros::ServiceServer record_pose;
    // ros::ServiceServer stop_move;

    ros::Subscriber poseSub;
    ros::Subscriber poseSubRight;
    ros::Subscriber calibrationSub;
    ros::Subscriber stopMoveSub;
    ros::Subscriber backHomeSub;

    ros::Publisher Object_pub;
    ros::Publisher planning_scene_diff_publisher;
    ros::Publisher pickPlace_pub; 

    graspBaseFun pickData;
    const double prepare_some_distance = 0.11;
    std::vector<std::string> pickObjectName;

    std::vector<std::vector<std::string> > detectionPosesName = {{"detect0ShelfTop", "detect0ShelfBottom", "detect0table"},
                                                    {"detect1ShelfTop", "detect1ShelfBottom", "detect1table"}};

    std::vector<std::vector<std::string> > placePosesName = {{"robot0table0", "robot0table1", "robot0shelf0", "robot0shelf1"},
                                                    {"robot1table0", "robot1table1", "robot1shelf0", "robot1shelf1"}};

    std::vector<std::vector<geometry_msgs::PoseStamped> > detectionPoses;
    std::vector<std::vector<geometry_msgs::PoseStamped> > placePoses;

    /*****************************************/
    recordLoadPose poseLoader;
    geometry_msgs::PoseStamped placePosesShelf1;
    int placePosesShelf1Cnt = 0;
    geometry_msgs::PoseStamped placePosesShelf2;
    int placePosesShelf2Cnt = 0;
    geometry_msgs::PoseStamped placePoseTable;
    int placePoseTableCnt = 0;
    int poseCount = 3;
    float poseDistance = 0.1;
    /*****************************************/

    std::string pkgPath;
    std::string detectionPosesPath = "detectionPoses";
    std::string placePosesPath = "placePoses";
    // 为true时停止
    bool isStop;
    bool isBackHome;
    bool isDetection;
    bool isGrasp;
};
