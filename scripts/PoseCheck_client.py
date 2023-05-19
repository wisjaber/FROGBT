#!/usr/bin/env python3

import rospy
from tree_msgs.srv import PoseCheck, PoseCheckRequest, PoseCheckResponse
from geometry_msgs.msg import PoseStamped


def PoseCheck_client():
    rospy.init_node('PoseCheck_client')
    rospy.wait_for_service('PoseCheckService')

    try:
        PoseCheckService = rospy.ServiceProxy('PoseCheckService', PoseCheck)
        request = PoseCheckRequest()
        request.PoseCheck.header.frame_id="map"
        request.PoseCheck.pose.position.x = -2.5
        request.PoseCheck.pose.position.y=-0.5
        request.PoseCheck.pose.position.z=1.39
        request.PoseCheck.pose.orientation.x=0.69
        request.PoseCheck.pose.orientation.y=-0.0
        request.PoseCheck.pose.orientation.z=-0.7
        request.PoseCheck.pose.orientation.w=0.0
        response = PoseCheckService(request)
    
        rospy.loginfo("Pose Check: {}".format(response))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

if __name__ == "__main__":
    PoseCheck_client()
