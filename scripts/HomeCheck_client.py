#!/usr/bin/env python3

import rospy
from tree_msgs.srv import HomeCheck, HomeCheckRequest, HomeCheckResponse
from geometry_msgs.msg import PoseStamped


def HomeCheck_client():
    rospy.init_node('HomeCheck_client')
    rospy.wait_for_service('HomeCheck_service')

    try:
        HomeCheckService = rospy.ServiceProxy('HomeCheck_service', HomeCheck)
        request = HomeCheckRequest()
        response = HomeCheckService(request)
    
        rospy.loginfo("Home Check: {}".format(response))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

if __name__ == "__main__":
    HomeCheck_client()
