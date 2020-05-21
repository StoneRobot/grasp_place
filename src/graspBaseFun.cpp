#include "grasp_place/graspBaseFun.h"

GraspPlace::GraspPlace(ros::NodeHandle nodehandle, \
                                moveit::planning_interface::MoveGroupInterface& group0, \
                                moveit::planning_interface::MoveGroupInterface& group1)
:move_group0{group0},
move_group1{group1}
{
    nh = nodehandle;
    openGripper_client0 = nh.serviceClient<hirop_msgs::openGripper>("/UR51/openGripper");
    closeGripper_client0 = nh.serviceClient<hirop_msgs::closeGripper>("/UR51/closeGripper");
    openGripper_client1 = nh.serviceClient<hirop_msgs::openGripper>("/UR52/openGripper");
    closeGripper_client1 = nh.serviceClient<hirop_msgs::closeGripper>("/UR52/closeGripper");

    remove_object_client = nh.serviceClient<hirop_msgs::RemoveObject>("removeObject");
    show_object_client = nh.serviceClient<hirop_msgs::ShowObject>("showObject");
    list_generator_client = nh.serviceClient<hirop_msgs::listGenerator>("listGenerator");
    list_actuator_client = nh.serviceClient<hirop_msgs::listActuator>("listActuator");
    set_gen_actuator_client = nh.serviceClient<hirop_msgs::SetGenActuator>("setGenActuator");
    detection_client = nh.serviceClient<hirop_msgs::detection>("detection");

    Object_pub = nh.advertise<hirop_msgs::ObjectArray>("object_array", 1);
    getPickData  = nh.advertiseService("grep_set", &GraspPlace::getPickDataCallBack, this);

    pose_sub = nh.subscribe("/object_array", 1, &GraspPlace::objectCallback, this);

    move_group0.setMaxAccelerationScalingFactor(0.1);
    move_group1.setMaxAccelerationScalingFactor(0.1);
    move_group0.setMaxVelocityScalingFactor(0.1);
    move_group1.setMaxVelocityScalingFactor(0.1);
    move_group0.setGoalPositionTolerance(0.01);
    move_group1.setGoalPositionTolerance(0.01);
    move_group0.setGoalOrientationTolerance(0.01);
    move_group1.setGoalOrientationTolerance(0.01);

    setGenActuator();

}

void GraspPlace::rmObject()
{
    hirop_msgs::RemoveObject srv;
    if(remove_object_client.call(srv))
    {
        ROS_INFO_STREAM("remove object "<< (srv.response.isSetFinsh ? "Succeed" : "Faild"));
    }
    else
    {
        ROS_INFO("check \\removeObject service ");
    }
}

void GraspPlace::showObject(geometry_msgs::Pose pose)
{
    hirop_msgs::ShowObject srv;
    srv.request.objPose.header.frame_id = "base_link";
    srv.request.objPose.pose.position.x = pose.position.x;
    srv.request.objPose.pose.position.y = pose.position.y;
    srv.request.objPose.pose.position.z = pose.position.z;
    srv.request.objPose.pose.orientation.x = pose.orientation.x;
    srv.request.objPose.pose.orientation.y = pose.orientation.y;
    srv.request.objPose.pose.orientation.z = pose.orientation.z;
    srv.request.objPose.pose.orientation.w = pose.orientation.w;
    if(show_object_client.call(srv))
    {
        ROS_INFO_STREAM("show object "<< (srv.response.isSetFinsh ? "Succeed" : "Faild"));
    }
    else
    {
        ROS_INFO("check \\showObject service ");
    }
}

bool GraspPlace::setGenActuator()
{
    hirop_msgs::listGenerator list_generator_srv;
    hirop_msgs::listActuator list_actuator_srv;
    hirop_msgs::SetGenActuator set_gen_actuator_srv;
    if(list_generator_client.call(list_generator_srv) && list_actuator_client.call(list_actuator_srv))
    {
        set_gen_actuator_srv.request.generatorName = list_generator_srv.response.generatorList[0];
        set_gen_actuator_srv.request.actuatorName = list_actuator_srv.response.actuatorList[0];
        if(set_gen_actuator_client.call(set_gen_actuator_srv))
            return true;
    }
    return false;
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

void GraspPlace::robotMoveCartesianUnit2(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z)
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
    while( group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) < 1.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
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
   bool flag = true;
   hirop_msgs::closeGripper srv;
   if(move_group.getName() == "arm0"){
       ROS_INFO("arm0 closeGripper ");
       flag = closeGripper_client0.call(srv);
   }
   else
   {
       ROS_INFO("arm1 closeGripper ");
       flag = closeGripper_client1.call(srv);
   }
    return flag;
}

bool GraspPlace::openGripper(moveit::planning_interface::MoveGroupInterface& move_group)
{
    bool flag = true;
   hirop_msgs::openGripper srv;
   if(move_group.getName() == "arm0"){
       flag = openGripper_client0.call(srv);
       ROS_INFO("arm0 openGripper ");
   }
   else
   {
       ROS_INFO("arm1 openGripper ");
       flag = openGripper_client1.call(srv);
   }
    return flag;
}

void GraspPlace::PickPlace(int robotNum, geometry_msgs::PoseStamped& pose, bool isPick, int pre_grasp_approach[], int post_grasp_retreat[])
{
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
        // 寸进的过程 后退
        robotMoveCartesianUnit2(getMoveGroup(robotNum), post_grasp_retreat[0]*prepare_some_distance, \
                                post_grasp_retreat[1]*prepare_some_distance, post_grasp_retreat[2]*prepare_some_distance);
        ROS_INFO("PICK UP  THE CUBE Z");
    }
    else
    {
        //Y AXIS
        robotMoveCartesianUnit2(getMoveGroup(robotNum), pre_grasp_approach[0]*prepare_some_distance, \
                                pre_grasp_approach[1]*prepare_some_distance, pre_grasp_approach[2]*prepare_some_distance);
        openGripper(getMoveGroup(robotNum));
        robotMoveCartesianUnit2(getMoveGroup(robotNum), post_grasp_retreat[0]*prepare_some_distance, \
                                post_grasp_retreat[1]*prepare_some_distance, post_grasp_retreat[2]*prepare_some_distance);
        ROS_INFO("PLACE UP  THE CUBE Y");
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
    while (ros::ok())
    {
        if(move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            break;
        }
    }
    return loop_move(move_group);
}

moveit::planning_interface::MoveItErrorCode GraspPlace::loop_move(moveit::planning_interface::MoveGroupInterface& move_group)
{
    int cnt = 0;
    moveit::planning_interface::MoveItErrorCode code;
    do
    {
        code = move_group.move();
        cnt ++;
    }
    while (ros::ok() && cnt < 10 && code.val == moveit::planning_interface::MoveItErrorCode::TIMED_OUT);
    return code;
}

void GraspPlace::getPickDataCallBack(grasp_place::rb_ArrayAndBool::Request& req, grasp_place::rb_ArrayAndBool::Response& rep)
{
    pickData.pickObject = req.data[1];
    pickData.pickRobot = req.data[2];
    pickData.pick_mode = req.data[3];
}

void GraspPlace::detectionObject(int objectNum)
{
    hirop_msgs::detection det_srv;
    det_srv.request.objectName = pickObjectName[objectNum];
    det_srv.request.detectorName = "Yolo6d";
    det_srv.request.detectorType = 1;
    det_srv.request.detectorConfig = "";
    if(detection_client.call(det_srv))
    {
        ROS_INFO_STREAM("detection result is " << det_srv.response.result);
        ros::WallDuration(2.0).sleep();
    }
    else
    {
        ROS_INFO("check detection service!!");
    }
}

void GraspPlace::objectCallback(const hirop_msgs::ObjectArray::ConstPtr& msg)
{
    
}

