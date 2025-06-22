#!/usr/bin/env python

import rospy
from my_rb1_ros.srv import Rotate, RotateResponse
from RB1 import RB1

def rotate_callback(request):
    rospy.loginfo("Service Requested")
    rotate_rb1_object = RB1()
    status = rotate_rb1_object.rotate(request.degrees)
    rospy.loginfo("Service Completed")
    response = RotateResponse()
    if status:
        response.result = "Rotation Completed Succesfully"
    else:
        response.result = "Rotation Failed"
    return response

rospy.init_node('RB1_server', anonymous=True) 
my_service = rospy.Service('/rotate_robot', Rotate, rotate_callback)
rospy.loginfo("Service Ready")
rospy.spin()