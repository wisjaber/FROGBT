#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from tree_msgs.srv import VacuumCheck, VacuumCheckRequest,VacuumCheckResponse
from franka_vacuum_gripper.msg import *


class VacuumMonitor():
    def __init__(self):
        rospy.init_node('VacuumCheck_server')
        self.vacuum_state_subscriber = rospy.Subscriber(
            "/franka_vacuum_gripper/vacuum_state", VacuumState, self.vacuum_state_cb
        )
        rospy.Service('VacuumCheckService', VacuumCheck, self.handle_vacuum_check)
        rospy.loginfo("Vacuum Check service is ready.")
        
    def vacuum_state_cb(self, msg: VacuumState):
        self._vacuum_state = msg

    def handle_vacuum_check(self,req):
        # Get the current vacuum state
        if self._vacuum_state.part_present:
            response = True
        else:
            response = False

        rospy.loginfo("Vacuum Check Service: {}".format(response))
        return VacuumCheckResponse(response)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    vacuum_service = VacuumMonitor()
    vacuum_service.run()
