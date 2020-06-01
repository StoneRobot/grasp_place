#include "grasp_place/graspBaseFun.h"

GraspPlace::GraspPlace(ros::NodeHandle nodehandle, \
                                moveit::planning_interface::MoveGroupInterface& group0, \
                                moveit::planning_interface::MoveGroupInterface& group1)
:move_group0{group0},
move_group1{group1}
{ 
    nh = nodehandle;
    // 
    openGripper_client0 = nh.serviceClient<hirop_msgs::openGripper>("/UR51/openGripper");
    closeGripper_client0 = nh.serviceClient<hirop_msgs::closeGripper>("/UR51/closeGripper");
    openGripper_client1 = nh.serviceClient<hirop_msgs::openGripper>("/UR52/openGripper");
    closeGripper_client1 = nh.serviceClient<hirop_msgs::closeGripper>("/UR52/closeGripper");

    detection_client = nh.serviceClient<hirop_msgs::detection>("UR51/detection");
    detection_client_right = nh.serviceClient<hirop_msgs::detection>("UR52/detection");

    Object_pub = nh.advertise<hirop_msgs::ObjectArray>("object_array", 1);
    planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    getPickData  = nh.advertiseService("/Rb_grepSetCommand", &GraspPlace::getPickDataCallBack, this);
    // stop_move = nh.advertiseService("stop_move", &GraspPlace::sotpMoveCallBack, this);
 
    poseSub = nh.subscribe("/UR51/object_array", 1, &GraspPlace::objectCallBack, this);
    poseSubRight = nh.subscribe("UR52/object_array", 1, &GraspPlace::objectCallBack, this);
    calibrationSub = nh.subscribe("calibration", 1, &GraspPlace::calibrationCallBack, this);
    stopMoveSub = nh.subscribe("/stop_move", 1, &GraspPlace::sotpMoveCallback, this);

    move_group0.setMaxAccelerationScalingFactor(0.1);
    move_group1.setMaxAccelerationScalingFactor(0.1);
    move_group0.setMaxVelocityScalingFactor(0.1);
    move_group1.setMaxVelocityScalingFactor(0.1);
    move_group0.setGoalPositionTolerance(0.001);
    move_group1.setGoalPositionTolerance(0.001);
    move_group0.setGoalOrientationTolerance(0.001);
    move_group1.setGoalOrientationTolerance(0.001);

    nh.getParam("/grasp_place/pkg_path", pkgPath);
    isStop = false;
    const int PoseCount = 3;
    detectionPoses.resize(2);
    // 0 上層貨架, 1 下層貨架, 2 桌子
    detectionPoses[0].resize(PoseCount);
    detectionPoses[1].resize(PoseCount);

    placePoses.resize(2);
    placePoses[0].resize(4);
    placePoses[1].resize(4);
    bool isLoadPose;
    nh.getParam("/grasp_place/isLoadPose", isLoadPose);
    if(isLoadPose)
    {
        loadPose(detectionPosesPath, detectionPosesName, detectionPoses);
        loadPose(placePosesPath, placePosesName, placePoses);
        loadPickObjectName();
        preprocessingPlacePose();
    }
}


void GraspPlace::showObject(geometry_msgs::Pose pose)
{
    std::vector<moveit_msgs::CollisionObject> collisionObject;
    collisionObject.resize(1);
    collisionObject[0].id = "object";
    collisionObject[0].header.frame_id = "world";
    collisionObject[0].primitives.resize(1);
    collisionObject[0].primitives[0].type = collisionObject[0].primitives[0].BOX;
    collisionObject[0].primitives[0].dimensions.resize(3);
    //"2 2 0.01 0 0.5 0.05
    collisionObject[0].primitives[0].dimensions[0] = 0.033;
    collisionObject[0].primitives[0].dimensions[1] = 0.033;
    collisionObject[0].primitives[0].dimensions[2] = 0.07;

    collisionObject[0].primitive_poses.resize(1);
    collisionObject[0].primitive_poses[0].orientation.x = pose.orientation.x;
    collisionObject[0].primitive_poses[0].orientation.y = pose.orientation.y;
    collisionObject[0].primitive_poses[0].orientation.z = pose.orientation.z;
    collisionObject[0].primitive_poses[0].orientation.w = pose.orientation.w;
    collisionObject[0].primitive_poses[0].position.x = pose.position.x;
    collisionObject[0].primitive_poses[0].position.y = pose.position.y;
    collisionObject[0].primitive_poses[0].position.z = pose.position.z;

    collisionObject[0].operation = moveit_msgs::CollisionObject::ADD;
    moveit_msgs::PlanningScene p;
    p.world.collision_objects.push_back(collisionObject[0]);
    p.is_diff = true;
    p.robot_state.is_diff = true;
    planning_scene_diff_publisher.publish(p);
    ros::Duration(0.1).sleep();
}


bool GraspPlace::transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id="world")
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::PoseStamped* worldFramePose = new geometry_msgs::PoseStamped[1];
    geometry_msgs::PoseStamped* otherFramePose = new geometry_msgs::PoseStamped[1];
    tf::TransformListener listener;

    otherFramePose[0] = poseStamped;
    for(int i=0; i < 5; ++i)
    {
        try
        {
            listener.transformPose(frame_id, otherFramePose[0], worldFramePose[0]);
            break;
        }
        catch(tf::TransformException& ex)
        {
            ROS_INFO_STREAM(ex.what());
            ros::WallDuration(1).sleep();
            continue;
        }
    }
    poseStamped = worldFramePose[0];
    delete[] worldFramePose;
    delete[] otherFramePose;
    if(poseStamped.header.frame_id == "world")
    {
        return true;
    }
    else
    {
        return false;
    }
}

void GraspPlace:: robotMoveCartesianUnit2(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z)
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
    while( group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) < 1.0 && !isStop);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    if(!isStop)
        group.execute(plan);
}

moveit::planning_interface::MoveGroupInterface& GraspPlace::getMoveGroup(int num)
{
    if(num == 0)
        return move_group0;
    else if (num == 1)
        return move_group1;
}

bool GraspPlace::closeGripper(moveit::planning_interface::MoveGroupInterface& move_group)
{
   bool flag = false;
   hirop_msgs::closeGripper srv;
   if(!isStop)
   {
        if(move_group.getName() == "arm0")
        {
            ROS_INFO("arm0 closeGripper ");
            flag = closeGripper_client0.call(srv);
        }
        else
        {
            ROS_INFO("arm1 closeGripper ");
            flag = closeGripper_client1.call(srv);
        }
   }
    return flag;
}

bool GraspPlace::openGripper(moveit::planning_interface::MoveGroupInterface& move_group)
{
    bool flag = false;
   hirop_msgs::openGripper srv;
   if(!isStop)
   {
        if(move_group.getName() == "arm0")
        {
            flag = openGripper_client0.call(srv);
            ROS_INFO("arm0 openGripper ");
        }
        else
        {
            ROS_INFO("arm1 openGripper ");
            flag = openGripper_client1.call(srv);
        }
   }
    return flag;
}

void GraspPlace::PickPlace(int robotNum, geometry_msgs::PoseStamped& pose, bool isPick, int pre_grasp_approach[], int post_grasp_retreat[])
{
    //TODO bug 1 去到放置目标点
    std::vector<double> joint = getMoveGroup(robotNum).getCurrentJointValues();
    moveit_msgs::RobotState r;
    r.joint_state.position = joint;
    getMoveGroup(robotNum).setStartState(r);
    getMoveGroup(robotNum).setStartStateToCurrentState();
    ros::Duration(0.1).sleep();
    
    setAndMove(getMoveGroup(robotNum), pose);
    if(isPick)
    { 
        //Z AXIS
        ROS_INFO("openGripper --- 1");
        openGripper(getMoveGroup(robotNum));
        ros::Duration(1).sleep();
        // 寸进的过程 前进1
        robotMoveCartesianUnit2(getMoveGroup(robotNum), pre_grasp_approach[0]*prepare_some_distance, \
                                pre_grasp_approach[1]*prepare_some_distance, pre_grasp_approach[2]*prepare_some_distance);

        ROS_INFO("closeGripper --- 2");
        closeGripper(getMoveGroup(robotNum));
        // 抓住物體
        getMoveGroup(robotNum).attachObject("object");
        // 寸进的过程 后退
        robotMoveCartesianUnit2(getMoveGroup(robotNum), post_grasp_retreat[0]*prepare_some_distance, \
                                post_grasp_retreat[1]*prepare_some_distance, post_grasp_retreat[2]*prepare_some_distance);
        // ROS_INFO("PICK UP  THE CUBE Z");
    }
    else
    {
        //Y AXIS
        robotMoveCartesianUnit2(getMoveGroup(robotNum), pre_grasp_approach[0]*prepare_some_distance, \
                                pre_grasp_approach[1]*prepare_some_distance, pre_grasp_approach[2]*prepare_some_distance);
        openGripper(getMoveGroup(robotNum));
        getMoveGroup(robotNum).detachObject("object");
        robotMoveCartesianUnit2(getMoveGroup(robotNum), post_grasp_retreat[0]*prepare_some_distance, \
                                post_grasp_retreat[1]*prepare_some_distance, post_grasp_retreat[2]*prepare_some_distance);
        // ROS_INFO("PLACE UP  THE CUBE Y");
    }
}


moveit::planning_interface::MoveItErrorCode GraspPlace::setAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                        geometry_msgs::PoseStamped& poseStamped)
{
    std::vector<double> joint = move_group.getCurrentJointValues();
    moveit_msgs::RobotState r;
    r.joint_state.position = joint;
    move_group.setStartState(r);
    move_group.setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPoseTarget(poseStamped);
    this->moveGroupPlanAndMove(move_group, my_plan);
}

moveit::planning_interface::MoveItErrorCode GraspPlace::moveGroupPlanAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
{
    int cnt = 0;
    moveit::planning_interface::MoveItErrorCode code;
    if(!isStop)  
    do
    {
        code = move_group.plan(my_plan);
    }
    while (ros::ok() && cnt < 5 && code.val != moveit::planning_interface::MoveItErrorCode::SUCCESS && !isStop);
    if(code.val == moveit::planning_interface::MoveItErrorCode::SUCCESS && !isStop)
    {
        code = loop_move(move_group);
    }
    return code;
}

moveit::planning_interface::MoveItErrorCode GraspPlace::loop_move(moveit::planning_interface::MoveGroupInterface& move_group)
{
    int cnt = 0;
    moveit::planning_interface::MoveItErrorCode code;
    do
    {
        ROS_INFO_STREAM("loop_move");
        code = move_group.move();
        cnt ++;
    }
    while (ros::ok() && cnt < 5 && code.val == moveit::planning_interface::MoveItErrorCode::TIMED_OUT && !isStop==false);
    return code;
}

void GraspPlace::backHome()
{
    if(!isStop)
    {
        ROS_INFO_STREAM("back home");
        const std::string home = "home" + std::to_string(pickData.pickRobot);
        getMoveGroup(pickData.pickRobot).setNamedTarget(home);
        getMoveGroup(pickData.pickRobot).move();
    }
}

bool GraspPlace::getPickDataCallBack(rb_msgAndSrv::rb_ArrayAndBool::Request& req, rb_msgAndSrv::rb_ArrayAndBool::Response& rep)
{
    nh.setParam("/isRuning_grab", true);
    bool isGetObject;
    ROS_INFO_STREAM("into callback function");
    // 从什么地方放到什么地方
    pickData.pickMode = req.data[0];
    // 捡什么东西
    pickData.pickObject = req.data[1];
    // 在指定机器人
    pickData.pickRobot = req.data[2];
    ROS_INFO_STREAM("Object: " << pickObjectName[pickData.pickObject] << ", robot: " << pickData.pickRobot << ", pickMode: " << pickData.pickMode);
    removeOrAddObject();
    rep.respond = true;
    ROS_INFO_STREAM("action ....");
    if(pickData.pickMode == 0)
    {
        // 貨架頂層
        setAndMove(getMoveGroup(pickData.pickRobot), detectionPoses[pickData.pickRobot][0]);
        ROS_INFO_STREAM(detectionPoses[pickData.pickRobot][0]);
        ROS_INFO_STREAM("to shelf top");
        if(detectionObject(pickData.pickObject, pickData.pickRobot))
        {
            nh.getParam("/grasp_place/isGetObject", isGetObject);
            // 没检测到时
            if(!isGetObject)
            {
                ROS_INFO_STREAM("to shelf buttom");
                // 貨架底層
                setAndMove(getMoveGroup(pickData.pickRobot), detectionPoses[pickData.pickRobot][1]);
                ROS_INFO_STREAM(detectionPoses[pickData.pickRobot][1]);
                if(detectionObject(pickData.pickObject, pickData.pickRobot))
                {
                    nh.getParam("/grasp_place/isGetObject", isGetObject);
                    if(!isGetObject)
                    {
                        // 不成功就回home點
                        backHome();
                        rep.respond = false;
                    }
                }
            }
        }    
    }
    else
    {
        // 檢測桌子上的东西
        setAndMove(getMoveGroup(pickData.pickRobot), detectionPoses[pickData.pickRobot][2]);
        ROS_INFO_STREAM(detectionPoses[pickData.pickRobot][2]);
        ROS_INFO_STREAM("to table");
        if(detectionObject(pickData.pickObject, pickData.pickRobot))
        {
            nh.getParam("/grasp_place/isGetObject", isGetObject);
            if(!isGetObject)
            {
                backHome();
            }
            rep.respond = false;
        }
    }
    nh.setParam("/grasp_place/isGetObject", false);
    if(rep.respond == false)
    {
        // 运动结束反馈
        nh.setParam("/isRuning_grab", false);
        ROS_INFO_STREAM("set 'isStop' false");
        isStop = false;
        rmObject("object");
    }
    return rep.respond;
}

void GraspPlace::objectCallBack(const hirop_msgs::ObjectArray::ConstPtr& msg)
{
    nh.setParam("/grasp_place/isGetObject", true);
    std::string home = "home";
    for(std::size_t i=0; i < msg->objects.size() && !isStop; ++i)
    {
        geometry_msgs::PoseStamped pose = msg->objects[i].pose;
        ROS_INFO_STREAM("not transform pick pose: " << pose);
        transformFrame(pose);
        ROS_INFO_STREAM("transform pick pose: " << pose);
        // system();
        pickPlaceObject(pose);
    }
    rmObject("wall");
    // 运动结束反馈
    isStop = false;
    nh.setParam("/isRuning_grab", false);
}

void GraspPlace::pickPlaceObject(geometry_msgs::PoseStamped pickPose)
{
    pick(pickPose);
    place();
    rmObject("wall");
    backHome();
}

bool GraspPlace::detectionObject(int objectNum, int robot)
{
    bool isFinish;
    hirop_msgs::detection det_srv;
    det_srv.request.objectName = pickObjectName[objectNum];
    det_srv.request.detectorName = "Yolo6d";
    det_srv.request.detectorType = 1;
    det_srv.request.detectorConfig = "";
    bool flag;
    if(robot==0)
        flag = detection_client.call(det_srv);
    else
        flag = detection_client_right.call(det_srv);
    if(flag)
    {
        ROS_INFO_STREAM("detection result is " << det_srv.response.result);
        isFinish = true;
    }
    else
    {
        ROS_INFO("check detection service!!");
        isFinish = false;
    }
    return isFinish;
}



void GraspPlace::calibrationCallBack(const std_msgs::Int8::ConstPtr& msg)
{
    switch (msg->data)
    {
    case 0:
        calibrationDetection(true);
        break;
    case 1:
        calibrationPlace(true);
        break;
    case 2:
        calibrationDetection(false);
        break;
    case 3:
        calibrationPlace(false);
        break;
    }
}

void GraspPlace::pick(geometry_msgs::PoseStamped pose)
{
    tf2::Quaternion orientation;
    // 寸进的方向,及多少倍的距离
    int pre_grasp_approach[3]={0};
    int post_grasp_retreat[3]={0};
    if(pickData.pickMode == 0)
    {
        orientation.setRPY(0, 0, 3.14);
        pose.pose.orientation = tf2::toMsg(orientation);
        pose.pose.position.x += prepare_some_distance;
        pre_grasp_approach[0] = -1;
        post_grasp_retreat[0] = 1;
    }
    else
    {
        orientation.setRPY(0, 0, pow(-1, pickData.pickRobot)*1.57);
        pose.pose.orientation = tf2::toMsg(orientation);
        pose.pose.position.y -= pow(-1, pickData.pickRobot)*prepare_some_distance;
        pre_grasp_approach[1] = pow(-1, pickData.pickRobot);
        post_grasp_retreat[2] = 1;
    }
    ROS_INFO_STREAM("pick pose: " << pose);
    showObject(pose.pose);
    PickPlace(pickData.pickRobot, pose, true, pre_grasp_approach, post_grasp_retreat);
}

void GraspPlace::place()
{
    // 每個機器人在桌子和貨架都有兩個擺放的位置, 這個變量是用來改變擺放位置的
    static int robotPlacePoseExchang[2][2]={{0, 2}, {0, 2}};
    int pre_grasp_approach[3]={0}; 
    int post_grasp_retreat[3]={0};
    // 放置
    if(pickData.pickMode == 0)
    {
        pre_grasp_approach[1] = pow(-1, pickData.pickRobot);
        post_grasp_retreat[2] = 1;
    }
    else
    {
        pre_grasp_approach[0] = -1;
        post_grasp_retreat[0] = 1;
    }
    PickPlace(pickData.pickRobot, placePoses[pickData.pickRobot][robotPlacePoseExchang[pickData.pickRobot][pickData.pickMode]], false, \
                pre_grasp_approach, post_grasp_retreat);

    ROS_INFO_STREAM("place: " << placePoses[pickData.pickRobot][robotPlacePoseExchang[pickData.pickRobot][pickData.pickMode]]);
    ROS_INFO_STREAM("i: " << pickData.pickRobot << " j: " << robotPlacePoseExchang[pickData.pickRobot][pickData.pickMode]);
    robotPlacePoseExchang[pickData.pickRobot][pickData.pickMode] =  (++robotPlacePoseExchang[pickData.pickRobot][pickData.pickMode])%2 \
                                                                    + pickData.pickMode * 2;
}

bool GraspPlace::writePoseOnceFile(const std::string& name, const geometry_msgs::PoseStamped& pose)
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

bool GraspPlace::addData(geometry_msgs::PoseStamped& pose, YAML::Node node)
{
    try
    {
        pose.header.frame_id = node["header"]["frame_id"].as<std::string>();
        pose.pose.position.x = node["pose"]["position"]["x"].as<double>();
        pose.pose.position.y = node["pose"]["position"]["y"].as<double>();
        pose.pose.position.z = node["pose"]["position"]["z"].as<double>();
        pose.pose.orientation.x = node["pose"]["orientation"]["x"].as<double>();
        pose.pose.orientation.y = node["pose"]["orientation"]["y"].as<double>();
        pose.pose.orientation.z = node["pose"]["orientation"]["z"].as<double>();
        pose.pose.orientation.w = node["pose"]["orientation"]["w"].as<double>();
        return true;
    }
    catch(const std::exception& e)
    {
        ROS_INFO_STREAM(e.what());
        // std::cerr << e.what() << '\n';
        return false;
    }
}

bool GraspPlace::recordPose(int robotNum, std::string name, bool isJointSpace, std::string folder="recordPose")
{
    std::string path;
    path = pkgPath + "/" + folder + "/" + name + ".yaml";
    if(isJointSpace)
    {
        ROS_INFO_STREAM("record joint space in development ...");
    }
    else
    {
        geometry_msgs::PoseStamped pose;
        getMoveGroup(robotNum).getCurrentPose();
        pose = getMoveGroup(robotNum).getCurrentPose();
        ROS_INFO_STREAM(pose);
        writePoseOnceFile(path, pose);
    }
    return true;
}

bool GraspPlace::loadPose(std::string folder, std::vector<std::vector<std::string> > filePathParam, std::vector<std::vector<geometry_msgs::PoseStamped> >& pose)
{
    std::string path;
    YAML::Node doc;
    std::size_t count = filePathParam[0].size();

    bool isFinish;
    for(std::size_t i=0; i<2; ++i)
    {
        for(std::size_t j=0; j<count; ++j)
        {
            path.clear();
            path = pkgPath + "/" + folder + "/" + filePathParam[i][j] + ".yaml";
            ROS_INFO_STREAM("load pose: " << path);
            doc = YAML::LoadFile(path);
            isFinish = addData(pose[i][j], doc);
            // ROS_INFO_STREAM(pose[i][j]);

            if(!isFinish)
                return isFinish;
        }
    }
    return isFinish;
}

bool GraspPlace::loadPickObjectName()
{
    std::string namePrefix = "detectionObject";
    std::string paramName;
    std::string objectName;
    int detectionObjectNum=0;
    nh.getParam("/grasp_place/detectionObjectNum", detectionObjectNum);
    for(std::size_t i=0; i<detectionObjectNum; ++i)
    {
        paramName = namePrefix + std::to_string(i);
        nh.getParam("/grasp_place" + paramName, objectName);
        pickObjectName.push_back(objectName);
        ROS_INFO_STREAM(pickObjectName[i]);
    }
}

void GraspPlace::calibration(std::vector<std::vector<geometry_msgs::PoseStamped> > pose, std::vector<std::vector<std::string> > poseName, std::string folder, bool isNowHavePoseFile)
{

    for(std::size_t i=0; i<poseName.size(); ++i)
    {
        for(std::size_t j=0; j<poseName[0].size() && ros::ok(); ++j)
        {
            if(isNowHavePoseFile)
                setAndMove(getMoveGroup(i), pose[i][j]);
            ROS_INFO_STREAM("robot: " << i << " calibration: " <<poseName[i][j]);
            ROS_INFO_STREAM("Press 'enter' to continue");
            std::cin.ignore();
            recordPose(i, poseName[i][j], false, folder);
        }
    }
}

void GraspPlace::calibrationDetection(bool isNowHavePoseFile)
{
    calibration(detectionPoses, detectionPosesName, "detectionPoses", isNowHavePoseFile);
}

void GraspPlace::calibrationPlace(bool isNowHavePoseFile)
{
    calibration(placePoses, placePosesName, "placePoses", isNowHavePoseFile);
}

void GraspPlace::preprocessingPlacePose()
{
    for(std::size_t i=0; i<placePoses.size(); ++i)
    {
        for(std::size_t j=0; j<placePoses[0].size(); ++j)
        {
            if(j < 2)
            {
                placePoses[i][j].pose.position.y -= pow(-1, i)*prepare_some_distance;
            }
            else
            {
                placePoses[i][j].pose.position.x += prepare_some_distance;
            }
        }
    }
}


void GraspPlace::rmObject(std::string name)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = name;
    collision_objects[0].operation = collision_objects[0].REMOVE;
    planning_scene_interface.applyCollisionObjects(collision_objects);
}


void GraspPlace::removeOrAddObject()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    rmObject("wall");

    collision_objects[0].id = "wall";
    collision_objects[0].header.frame_id = "base_link";

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    //"2 2 0.01 0 0.5 0.05
    collision_objects[0].primitives[0].dimensions[0] = 1;
    collision_objects[0].primitives[0].dimensions[1] = 0.01;
    collision_objects[0].primitives[0].dimensions[2] = 1;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = 0.5 + pow(-1, pickData.pickRobot)*0.3;
    collision_objects[0].primitive_poses[0].position.z = 0.5;

    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    collision_objects[0].primitive_poses[0].orientation = tf2::toMsg(orientation);

    collision_objects[0].operation = collision_objects[0].ADD;

    moveit_msgs::PlanningScene p;
    p.world.collision_objects.push_back(collision_objects[0]);
    p.is_diff = true;
    p.robot_state.is_diff = true;
    // p.object_colors.push_back();
    planning_scene_diff_publisher.publish(p);
}

void GraspPlace::stopMove()
{
    ROS_INFO_STREAM("stop ...");
    move_group0.stop();
    move_group1.stop();
    isStop = true;
}

void GraspPlace::sotpMoveCallback(const std_msgs::Bool::ConstPtr& msg)
{
    stopMove();
}
