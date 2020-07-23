#! /usr/bin/env python
#coding=utf-8

import rospy
from rb_msgAndSrv.srv import rb_ArrayAndBool, rb_ArrayAndBoolRequest, rb_ArrayAndBoolResponse

rospy.init_node("cli_test")
client = rospy.ServiceProxy("Rb_grepSetCommand", rb_ArrayAndBool)
# a = rb_ArrayAndBool()
# a.data = 
print(client.call([0, 2, 0, 0]))