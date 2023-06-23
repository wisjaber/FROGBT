#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from tree_msgs.srv import LocationCheck, LocationCheckRequest,LocationCheckResponse


def LocationCheck_client():
    rospy.init_node('LocationCheck_client')
    rospy.wait_for_service('LocationCheck_Service')

    try:
        LocationCheckService = rospy.ServiceProxy('LocationCheck_Service', LocationCheck)
        request = Pose()
        request.position.x = 0.0
        request.position.y=-0.0
        # request.position.z=0.0
        # request.orientation.x=0.0
        # request.orientation.y=-0.0
        request.orientation.z=0.99
        # request.orientation.w=0.0
        response = LocationCheckService(request)
    
        rospy.loginfo("Location Check: {}".format(response))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

if __name__ == "__main__":
    LocationCheck_client()
