#!/usr/bin/env python3

import rospy
from tree_msgs.srv import VacuumCheck

def vacuum_check_client():
    rospy.init_node('VacuumCheck_client')
    rospy.wait_for_service('VacuumCheckService')

    try:
        vacuum_check = rospy.ServiceProxy('VacuumCheckService', VacuumCheck)
        response = vacuum_check()
        rospy.loginfo("Vacuum State: {}".format(response))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

if __name__ == "__main__":
    vacuum_check_client()
