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

    // Object_pub = nh.advertise<hirop_msgs::ObjectArray>("object_array", 1);
    planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    pickPlace_pub = nh.advertise<std_msgs::Bool>("grabOk", 10);

    getPickData  = nh.advertiseService("/Rb_grepSetCommand", &GraspPlace::getPickDataCallBack, this);
    record_pose = nh.advertiseService("record_grasp_place_pose", &GraspPlace::recordPoseCallBack, this);
    // stop_move = nh.advertiseService("stop_move", &GraspPlace::sotpMoveCallBack, this);
 
    poseSub = nh.subscribe("/UR51/object_array", 1, &GraspPlace::objectCallBack, this);
    poseSubRight = nh.subscribe("UR52/object_array", 1, &GraspPlace::objectCallBack, this);
    calibrationSub = nh.subscribe("calibration", 1, &GraspPlace::calibrationCallBack, this);
    stopMoveSub = nh.subscribe("/stop_move", 1, &GraspPlace::sotpMoveCallback, this);
    backHomeSub = nh.subscribe("/back_home", 1, &GraspPlace::backHomeCallback, this);

    double speed;
    nh.param("/grasp_place/speed", speed, 0.5);
    // move_group0.setMaxAccelerationScalingFactor(speed);
    // move_group1.setMaxAccelerationScalingFactor(speed);
    move_group0.setMaxVelocityScalingFactor(speed);
    move_group1.setMaxVelocityScalingFactor(speed);
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
    /*******************************************/
    std::string path = pkgPath + "/placePoses/";
    ROS_INFO(path.c_str());
    poseLoader.loadRobotPose(placePosesShelf1, path + "placePosesShelf1.yaml");
    poseLoader.loadRobotPose(placePosesShelf2, path + "placePosesShelf2.yaml");
    poseLoader.loadRobotPose(placePoseTable, path + "placePoseTable.yaml");
    nh.getParam("/grasp_place/poseDistance", poseDistance);
    /*******************************************/

    /*******************************************/
    placePoses.resize(2);
    placePoses[0].resize(4);
    placePoses[1].resize(4);
    bool isLoadPose;
    nh.getParam("/grasp_place/isLoadPose", isLoadPose);
    if(isLoadPose)
    {
        loadPose(detectionPosesPath, detectionPosesName, detectionPoses);
        ROS_INFO("---------------------1");
        loadPose(placePosesPath, placePosesName, placePoses);
        ROS_INFO("---------------------2");
        loadPickObjectName();
        ROS_INFO("---------------------3");
        preprocessingPlacePose();
        ROS_INFO("---------------------4");
    }
    /*******************************************/
    isGrasp = false;
    isDetection = false;
}

geometry_msgs::PoseStamped GraspPlace::changeOrientation(geometry_msgs::PoseStamped& pose, int RPY)
{
    tf2::Quaternion orientation;
    if(RPY == 0)
    {
        orientation.setRPY(0, 0, 3.14);
    }
    else if(RPY == 1)
    {
        orientation.setRPY(0, 0, 1.57);
    }
    else if(RPY == 2)
    {
        orientation.setRPY(0, 0, -1.57);
    }
    pose.pose.orientation = tf2::toMsg(orientation);
    return pose;
}

void GraspPlace::backHomeCallback(const std_msgs::Int8::ConstPtr& msg)
{
    stopAndBackHome(msg->data);
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
    collisionObject[0].primitives[0].dimensions[0] = 0.033;
    collisionObject[0].primitives[0].dimensions[1] = 0.033;
    collisionObject[0].primitives[0].dimensions[2] = 0.0806;

    collisionObject[0].primitive_poses.resize(1);
    collisionObject[0].primitive_poses[0].position.x = pose.position.x;
    collisionObject[0].primitive_poses[0].position.y = pose.position.y;
    collisionObject[0].primitive_poses[0].position.z = pose.position.z;
    collisionObject[0].primitive_poses[0].orientation.x = pose.orientation.x;
    collisionObject[0].primitive_poses[0].orientation.y = pose.orientation.y;
    collisionObject[0].primitive_poses[0].orientation.z = pose.orientation.z;
    collisionObject[0].primitive_poses[0].orientation.w = pose.orientation.w;
    collisionObject[0].operation = moveit_msgs::CollisionObject::ADD;

    moveit_msgs::PlanningScene p;
    p.world.collision_objects.push_back(collisionObject[0]);
    p.is_diff = true;
    p.robot_state.is_diff = true;
    planning_scene_diff_publisher.publish(p);
    ros::Duration(1).sleep();
}


bool GraspPlace::transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id="world")
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::PoseStamped* worldFramePose = new geometry_msgs::PoseStamped[1];
    geometry_msgs::PoseStamped* otherFramePose = new geometry_msgs::PoseStamped[1];
    tf::TransformListener listener;

    // poseStamped.pose.position.z = -poseStamped.pose.position.z;

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
    double add[3] = {0};
    nh.getParam("/grasp_place/position_x_add", add[0]);
    nh.getParam("/grasp_place/position_y_add", add[1]);
    nh.getParam("/grasp_place/position_z_add", add[2]);
    if(pickData.pickMode == 0)
        (poseStamped.pose.position.z > 1.50) ? poseStamped.pose.position.z = 1.612 : poseStamped.pose.position.z = 1.34;
    else
    {
            poseStamped.pose.position.z < 1.10 ? poseStamped.pose.position.z = 1.13 : poseStamped.pose.position.z = 1.13;
    }
        // poseStamped.pose.position.z = 1.632;
    poseStamped.pose.position.x += add[0];
    poseStamped.pose.position.y += add[1];
    poseStamped.pose.position.z += add[2];
    if(poseStamped.header.frame_id == "world")
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool GraspPlace:: robotMoveCartesianUnit2(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z)
{
    ros::Duration(1).sleep();
    std::vector<double>joint = group.getCurrentJointValues();
    moveit_msgs::RobotState r;
    r.joint_state.position = joint;
    group.setStartState(r);
    group.setStartStateToCurrentState();
    geometry_msgs::PoseStamped temp_pose = group.getCurrentPose();

    moveit::planning_interface::MoveItErrorCode code;
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
    int cntEnd = 10;
    while(group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) < 0.50 && !isStop && ros::ok() && ++cnt < cntEnd);
    // ROS_INFO_STREAM("computeCartesianPath: " << )
    if(cnt >= cntEnd)
        return false;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    if(!isStop)
    {
        code = group.execute(plan);
        return code2bool(code);
    }
    return false;
}

bool GraspPlace::code2bool(moveit::planning_interface::MoveItErrorCode code)
{
    if(code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        return true;
    else
    {
        return false;
    }
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
        ros::Duration(1.0).sleep();
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
        ros::Duration(1.0).sleep();
   }
    return flag;
}

bool GraspPlace::PickPlace(int robotNum, geometry_msgs::PoseStamped& pose, bool isPick, int pre_grasp_approach[], int post_grasp_retreat[])
{
    // setStartState(getMoveGroup(robotNum));
    ROS_ERROR_STREAM("----into PickPlace-----");
    ROS_INFO_STREAM(pose);
    // std::cin.ignore();
    if(move(getMoveGroup(robotNum), pose))
    {
        if(isPick)
        { 
            ROS_ERROR_STREAM("----into Pick-----");
            //Z AXIS
            ROS_INFO("openGripper --- 1");
            openGripper(getMoveGroup(robotNum));
            // 寸进的过程 前进1
            if(robotMoveCartesianUnit2(getMoveGroup(robotNum), pre_grasp_approach[0]*prepare_some_distance, \
                                    pre_grasp_approach[1]*prepare_some_distance, pre_grasp_approach[2]*prepare_some_distance))
            {
                ROS_INFO("closeGripper --- 2");
                closeGripper(getMoveGroup(robotNum));
                // 抓住物體
                getMoveGroup(robotNum).attachObject("object");
                // 寸进的过程 后退
                ROS_INFO("robotMoveCartesianUnit2 0 0 0.02");
                robotMoveCartesianUnit2(getMoveGroup(robotNum), 0, 0, 0.02);

                ROS_INFO("robotMoveCartesianUnit2 0 prepare_some_distance 0");
                robotMoveCartesianUnit2(getMoveGroup(robotNum), post_grasp_retreat[0]*prepare_some_distance, \
                            post_grasp_retreat[1]*prepare_some_distance, post_grasp_retreat[2]*prepare_some_distance);
                return true;

                // ROS_INFO("PICK UP  THE CUBE Z");
            }
        }
        else
        {
            ROS_ERROR_STREAM("----into Place-----");
            //Y AXIS
            if(robotMoveCartesianUnit2(getMoveGroup(robotNum), pre_grasp_approach[0]*prepare_some_distance, \
                                    pre_grasp_approach[1]*prepare_some_distance, pre_grasp_approach[2]*prepare_some_distance))
            {
                openGripper(getMoveGroup(robotNum));
                getMoveGroup(robotNum).detachObject("object");
                robotMoveCartesianUnit2(getMoveGroup(robotNum), post_grasp_retreat[0]*prepare_some_distance, \
                                        post_grasp_retreat[1]*prepare_some_distance, post_grasp_retreat[2]*prepare_some_distance);
                return true;
            }
        }
    }
    // openGripper(getMoveGroup(robotNum));
    return false;
}

bool GraspPlace::test(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::PoseStamped& poseStamped)
{
    move_group.setPoseTarget(poseStamped);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    return (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}


void GraspPlace::setStartState(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<double> joint = move_group.getCurrentJointValues();
    moveit_msgs::RobotState r;
    r.joint_state.position = joint;
    move_group.setStartState(r);
    move_group.setStartStateToCurrentState();
}

bool GraspPlace::move(moveit::planning_interface::MoveGroupInterface& move_group, \
                                geometry_msgs::PoseStamped& poseStamped)
{
    return code2bool(setAndMove(move_group, poseStamped));
}

moveit::planning_interface::MoveItErrorCode GraspPlace::setAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                        geometry_msgs::PoseStamped& poseStamped)
{
    setStartState(move_group);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPoseTarget(poseStamped);
    return this->moveGroupPlanAndMove(move_group, my_plan);
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
        cnt ++;
    }
    while (ros::ok() && cnt < 1 && code.val != moveit::planning_interface::MoveItErrorCode::SUCCESS && !isStop);
    if(code == moveit::planning_interface::MoveItErrorCode::SUCCESS && !isStop)
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
    while (ros::ok() && cnt < 2 && code == moveit::planning_interface::MoveItErrorCode::TIMED_OUT && !isStop==false);
    return code;
}

void GraspPlace::backHome(int robot=2)
{
    if(!isStop || isBackHome)
    {
        isBackHome = false;
        ROS_INFO_STREAM("back home");
        if(robot == 2)
            robot = pickData.pickRobot;
        const std::string home = "home" + std::to_string(robot);
        getMoveGroup(robot).setNamedTarget(home);
        getMoveGroup(robot).move();
    }
    nh.setParam("/isRuning_grab", false);
}

bool GraspPlace::getPickDataCallBack(rb_msgAndSrv::rb_ArrayAndBool::Request& req, rb_msgAndSrv::rb_ArrayAndBool::Response& rep)
{
    if(req.data[3] == 1)
    {
        ROS_ERROR_STREAM("getPickDataCallBack function --> back home");
        backHome();
        rep.respond = false;
        return false;
    }
    isStop = false;
    isDetection = true;
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
    rep.respond = 1;
    ROS_INFO_STREAM("action ....");
    if(pickData.pickMode == 0)
    {
        // 貨架頂層
        setAndMove(getMoveGroup(pickData.pickRobot), detectionPoses[pickData.pickRobot][0]);
        ROS_INFO_STREAM(detectionPoses[pickData.pickRobot][0]);
        ROS_INFO_STREAM("to shelf top");
        if(detectionObject(pickData.pickObject, pickData.pickRobot))
        {
            ros::Duration(1.0).sleep();
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
                    ros::Duration(1.0).sleep();
                    nh.getParam("/grasp_place/isGetObject", isGetObject);
                    if(!isGetObject)
                    {
                        rep.respond = 0;
                    }
                }
                else
                {
                    rep.respond = 0;
                }
            }
        }
        else
        {
            rep.respond = 0;
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
            ros::Duration(1.0).sleep();
            nh.getParam("/grasp_place/isGetObject", isGetObject);
            if(!isGetObject)
            {
                rep.respond = 0;
            }
        }
        else
        {
            rep.respond = 0;
        }
    }
    if(rep.respond == false)
    {
        // 运动结束反馈
        nh.setParam("/isRuning_grab", false);
        isStop = false;
        rmObject("object");
    }
    isDetection = false;
    bool checkFlag = rep.respond;
    ROS_ERROR_STREAM("rep.respond: " << checkFlag);
    nh.setParam("/grasp_place/isGetObject", false);
    return rep.respond;
}

void GraspPlace::objectCallBack(const hirop_msgs::ObjectArray::ConstPtr& msg)
{
    isGrasp = true;
    bool flag;
    nh.setParam("/grasp_place/isGetObject", true);
    std::string home = "home";
    std::string  cmd0 = "rosservice call /UR5" + std::to_string(pickData.pickRobot + 1) + "/set_robot_enable \"enable: false\"";
    std::string  cmd1 = "rosservice call /UR5" + std::to_string(pickData.pickRobot + 1) + "/set_robot_enable \"enable: true\"";
    for(std::size_t i=0; i < msg->objects.size() && !isStop && ros::ok(); ++i)
    {
        geometry_msgs::PoseStamped pose = msg->objects[i].pose;
        ROS_INFO_STREAM("not transform pick pose: " << pose);
        transformFrame(pose);
        ROS_INFO_STREAM("transform pick pose: " << pose);
        bool isOnlyShow;
        nh.getParam("/grasp_place/isOnlyShow", isOnlyShow);
        if(isOnlyShow)
            showObject(pose.pose);
        else
            flag = pickPlaceObject(pose);
    }
    rmObject("wall");
    // 运动结束反馈
    isStop = false;
    isGrasp = false;
    nh.setParam("/isRuning_grab", false);
    system(cmd0.c_str());
    system(cmd1.c_str());
    std_msgs::Bool m;
    m.data = flag;
    pickPlace_pub.publish(m);
    nh.setParam("/grasp_place/isGetObject", false);
}

bool GraspPlace::pickPlaceObject(geometry_msgs::PoseStamped pickPose)
{
    bool flag;
    if(pick(pickPose))
    {   
        ROS_INFO_STREAM("pick SUCCESS");
        if(place())
        {
            ROS_INFO_STREAM("place SUCCESS");
            flag = true;
        }
        else
        {
            ROS_INFO_STREAM("place FAILED");
            flag = false;
        }
    }
    else
    {
         ROS_INFO_STREAM("pick FAILED");
        flag = false;
    }
    getMoveGroup(pickData.pickRobot).detachObject("object");
    backHome();
    return flag;
}

bool GraspPlace::detectionObject(int objectNum, int robot)
{
    if(isStop)
        return false;
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

bool GraspPlace::pick(geometry_msgs::PoseStamped pose)
{
    tf2::Quaternion orientation;
    // 寸进的方向,及多少倍的距离
    int pre_grasp_approach[3]={0};
    int post_grasp_retreat[3]={0};
    if(pickData.pickMode == 0)
    {
        orientation.setRPY(0, 0, 3.14);
        pose.pose.orientation = tf2::toMsg(orientation);
        showObject(pose.pose);
        pose.pose.position.x += prepare_some_distance;
        pre_grasp_approach[0] = -1;
        post_grasp_retreat[0] = 1;
    }
    else
    {
        orientation.setRPY(0, 0, pow(-1, pickData.pickRobot)*1.57);
        pose.pose.orientation = tf2::toMsg(orientation);
        showObject(pose.pose);
        pose.pose.position.y -= pow(-1, pickData.pickRobot)*prepare_some_distance;
        pre_grasp_approach[1] = pow(-1, pickData.pickRobot);
        post_grasp_retreat[2] = 1;
    }
    ROS_INFO_STREAM("pick pose: " << pose);
    return PickPlace(pickData.pickRobot, pose, true, pre_grasp_approach, post_grasp_retreat);
}

bool GraspPlace::place()
{
    bool flag;
    // 每個機器人在桌子和貨架都有兩個擺放的位置, 這個變量是用來改變擺放位置的
    // static int robotPlacePoseExchang[2][2]={{0, 2}, {0, 2}};
    int pre_grasp_approach[3]={0}; 
    int post_grasp_retreat[3]={0};
    // 放置
    if(pickData.pickMode == 0)
    {
        pre_grasp_approach[1] = pow(-1, pickData.pickRobot);
        post_grasp_retreat[2] = 1;
        if(pickData.pickRobot == 0)
        {
            setAndMove(move_group0, detectionPoses[0][2]);
        }
        else
        {
            setAndMove(move_group1, detectionPoses[1][2]);
        }
    }
    else
    {
        pre_grasp_approach[0] = -1;
        post_grasp_retreat[0] = 1;
    }
    /******************* 生成坐标 *******************/
    // 0 从货架得到桌子
    // 1 从桌子到货架
    geometry_msgs::PoseStamped pose;
    int cntFlag = 0;
    if(pickData.pickMode == 0)
    {
        cntFlag = 1;
        pose = placePoseTable;
        pose.pose.position.x -= (placePoseTableCnt * poseDistance);
        placePoseTableCnt++;
        placePoseTableCnt %= poseCount;
        // * 0 货架
        // * 1 左机器人
        // * 2 右机器人
        if(pickData.pickRobot == 0)
        {
            changeOrientation(pose, 1);
            preprocessingPlacePose(pose, 1);
        }
        else
        {
            changeOrientation(pose, 2);
            preprocessingPlacePose(pose, 2);
        }
        ROS_INFO_STREAM("changeOrientation" << pose);
    }
    else
    {
        if(placePosesShelf1Cnt != poseCount)
        {
            cntFlag = 2;
            pose = placePosesShelf1;
            pose.pose.position.y += (placePosesShelf1Cnt * poseDistance);
            placePosesShelf1Cnt ++;
            setAndMove(getMoveGroup(pickData.pickRobot), detectionPoses[pickData.pickRobot][0]);
        }
        else
        {
            cntFlag = 3;
            pose = placePosesShelf2;
            pose.pose.position.y += (placePosesShelf2Cnt * poseDistance);
            placePosesShelf2Cnt++;
            setAndMove(getMoveGroup(pickData.pickRobot), detectionPoses[pickData.pickRobot][1]);
            if(placePosesShelf2Cnt == poseCount)
            {
                placePosesShelf1Cnt = 0;
                placePosesShelf2Cnt = 0;
            }
        }
        changeOrientation(pose, 0);
        preprocessingPlacePose(pose, 0);
    }
    ROS_INFO_STREAM("----->place pose---->" << pose);
    flag = PickPlace(pickData.pickRobot, pose, false, pre_grasp_approach, post_grasp_retreat);
    if(!flag)
    {
        switch (cntFlag)
        {
        case 1:
            placePoseTableCnt--;
            break;
        case 2:
            placePosesShelf1Cnt--;
            break;
        case 3:
            placePosesShelf2Cnt--;
            break;
        default:
            break;
        }
    }

    // flag = PickPlace(pickData.pickRobot, placePoses[pickData.pickRobot][robotPlacePoseExchang[pickData.pickRobot][pickData.pickMode]], false, \
    //             pre_grasp_approach, post_grasp_retreat);

    // ROS_INFO_STREAM("place: " << placePoses[pickData.pickRobot][robotPlacePoseExchang[pickData.pickRobot][pickData.pickMode]]);
    // ROS_INFO_STREAM("i: " << pickData.pickRobot << " j: " << robotPlacePoseExchang[pickData.pickRobot][pickData.pickMode]);
    // robotPlacePoseExchang[pickData.pickRobot][pickData.pickMode] =  (++robotPlacePoseExchang[pickData.pickRobot][pickData.pickMode])%2 \
    //                                                                 + pickData.pickMode * 2;
    return flag;
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

bool GraspPlace::recordPoseCallBack(rubik_cube_solve::recordPoseStamped::Request& req, rubik_cube_solve::recordPoseStamped::Response& rep)
{
    recordPose(req.robot, req.PoseName, req.isJointSpace, "recordPose");
    rep.isFinish = true;
    return true;
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
        nh.getParam("/grasp_place/" + paramName, objectName);
        pickObjectName.push_back(objectName);
        ROS_INFO_STREAM(pickObjectName[i]);
    }
}

void GraspPlace::calibration(std::vector<std::vector<geometry_msgs::PoseStamped> > pose, std::vector<std::vector<std::string> > poseName, std::string folder, bool isNowHavePoseFile)
{

    for(std::size_t i=0; i<poseName.size()&& ros::ok(); ++i)
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
    for(std::size_t i=0; i<placePoses.size()&& ros::ok(); ++i)
    {
        for(std::size_t j=0; j<placePoses[0].size()&& ros::ok(); ++j)
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

void GraspPlace::preprocessingPlacePose(geometry_msgs::PoseStamped& pose, int type)
{
    if(type == 0)
        pose.pose.position.x += prepare_some_distance;
    else if(type == 1)
        pose.pose.position.y -= prepare_some_distance;
    else if(type == 2)
        pose.pose.position.y += prepare_some_distance;
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
    collision_objects[0].primitive_poses[0].position.y = 0.5 + pow(-1, pickData.pickRobot)*0.28;
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
    nh.setParam("/isRuning_grab", false);
}

void GraspPlace::sotpMoveCallback(const std_msgs::Bool::ConstPtr& msg)
{
    isBackHome = false;
    stopMove();
}

void  GraspPlace::stopAndBackHome(int robotNum)
{
    stopMove();
    isBackHome = true;
    if(!isGrasp && !isDetection)
    {
        ROS_INFO("go to back home");
        backHome(robotNum);
        isBackHome = false;
    }
}
