#!/usr/bin/env python3
import rospy
import actionlib

from tree_msgs.msg import MoveArmAction, MoveArmActionGoal,MoveArmActionFeedback,MoveArmActionResult
from move_arm_planner import MoveArmPlanner

class MoveArmActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('move_arm_action', MoveArmAction, self.execute, False)
        self.server.start()

        self.planner_id = "RRTConnect"
        self.num_planning_attempts = 5
        self.plan_time_sec = 5.0

    def execute(self, goal):
        planner = MoveArmPlanner(self.planner_id, self.num_planning_attempts, self.plan_time_sec, debug=False)
        start_pos = [-0.0, -0.8, 0.0, -2.3, 0.0, 3.1, 0.8]
        result = MoveArmActionResult()
        try:
            planner.move_group.go(start_pos, wait=True)
            result.result.success = True
            self.server.set_succeeded(result.result)
        except Exception as e:
            rospy.logerr('Failed to execute move_group action: {}'.format(str(e)))
            result.result.success = False
            self.server.set_aborted(result.result)

if __name__ == '__main__':
    rospy.init_node("move_arm_server")
    server = MoveArmActionServer()
    rospy.spin()
