#! /usr/bin/env python
#coding=utf-8

import rospy
from hirop_msgs.srv import detection, detectionRequest, detectionResponse
from hirop_msgs.msg import ObjectArray, ObjectInfo
from geometry_msgs.msg import PoseStamped

rospy.init_node("GraspPlace_test")
array_object = rospy.Publisher("/UR51/object_array", ObjectArray, queue_size=10)
array_object_right = rospy.Publisher("/UR52/object_array", ObjectArray, queue_size=10)
cnt = 0
def callback(req):
    global cnt
    rospy.loginfo("light")
    pose_stamped = PoseStamped()
    pose_object_info = ObjectInfo()
    msg = ObjectArray()
    pose_stamped.header.frame_id = "pick_gripper_link_0"
    pose_stamped.pose.position.x = 0.2
    pose_stamped.pose.position.y = 0
    pose_stamped.pose.position.z = 0
    pose_stamped.pose.orientation.w = 1
    pose_object_info.pose = pose_stamped
    msg.objects.append(pose_object_info)
    # if cnt == 3:
    array_object.publish(msg)
    rospy.loginfo(cnt)
    cnt += 1
    detectionResponse(1)
    return True
    # rospy.logout(pose_stamped)

def callback_right(req):
    global cnt
    rospy.loginfo("right")
    pose_stamped = PoseStamped()
    pose_object_info = ObjectInfo()
    msg = ObjectArray()
    pose_stamped.header.frame_id = "pick_gripper_link_1"
    pose_stamped.pose.position.x = 0.2
    pose_stamped.pose.position.y = 0
    pose_stamped.pose.position.z = 0
    pose_stamped.pose.orientation.w = 1
    pose_object_info.pose = pose_stamped
    msg.objects.append(pose_object_info)
    # if cnt == 3:
    array_object_right.publish(msg)
    rospy.loginfo(cnt)
    cnt += 1
    detectionResponse(1)
    return True
    # rospy.logout(pose_stamped)

detectionn_ser = rospy.Service("/UR51/detection", detection, callback)
detectionn_ser = rospy.Service("/UR52/detection", detection, callback_right)
rospy.logout("...")
rospy.spin()
