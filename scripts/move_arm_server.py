#!/usr/bin/env python3

import sys
import rospy
import actionlib
import numpy as np
import tf
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import PoseStamped

# import custom classes
from move_arm_planner import MoveArmPlanner
from tree_msgs.msg import MoveArmAction, MoveArmActionGoal,MoveArmActionFeedback,MoveArmActionResult


class MoveArmActionServers(object):
    def __init__(self, action_name, sim=False, debug=False):
        
        self.action_name = action_name
        self.possible_actions = ["move"]
        
        rospy.loginfo("node: {} is initializing action servers: {}".format(
            rospy.get_name(), self.action_name))

        self.planner = self.init_move_arm_planner(sim=sim, debug=debug)
        self.possible_movements = self.planner.move_names
        
        self.move_server, self.move_feedback, self.move_result = self.init_move_server()
        self.move_server.start()
        
        rospy.loginfo("in node: {}, Action servers {}.".format(
            rospy.get_name(), self.action_name))
        
    def init_move_arm_planner(self, sim=False, debug=False):        

        planner_id = "RRTConnect"
        num_planning_attempts = 5
        plan_time_sec = 5.0
        self.planner = MoveArmPlanner(planner_id, num_planning_attempts, plan_time_sec, debug=debug)        
        return self.planner

    def init_move_server(self): 
        # init move_server
        self.plan_goal = MoveArmActionGoal()
        self.move_feedback = MoveArmActionFeedback()
        self.move_result = MoveArmActionResult()
        self.move_server = actionlib.SimpleActionServer(self.action_name, MoveArmAction, 
                                                   execute_cb=self.plan_cb, 
                                                   auto_start = False)
        return self.move_server, self.move_feedback, self.move_result
            
    def plan_cb(self, goal):
        start_time = rospy.get_time()
        
        success = False
        sec_passed = 0
        status = ""

        # check if valid goal.action input
        if not goal.action in self.possible_actions:
            status = "{} is not a valid input for 'action'. Choose either: {}".format(
                goal.action, self.possible_actions)
            rospy.loginfo(status)
            sec_passed = rospy.get_time() - start_time 
            self.move_result = self.set_output_msg(self.move_result, success, sec_passed, status)
            self.move_server.set_aborted(self.move_result)
            return

        # check if valid goal.movements input
        undefined_movements = [movement for movement in goal.movements if not movement in self.possible_movements]
        if len(undefined_movements) > 0:
            status = "{} is/are undefined movement(s). Choose either: {}".format(
                undefined_movements, self.possible_movements)
            rospy.loginfo(status)
            sec_passed = rospy.get_time() - start_time 
            self.move_result = self.set_output_msg(self.move_result, success, sec_passed, status)
            self.move_server.set_aborted(self.move_result)
            return

        # loop over the movements and do ether plan, move, or plan_and_move
        for movement in goal.movements:
            if self.move_server.is_preempt_requested():
                status = "preempt is requested by client" 
                rospy.loginfo(status)
                success = False
                sec_passed = rospy.get_time() - start_time 
                break 
                                    
            if goal.action == "plan_and_move":
                # do execution and return success
                success, status = self.planner.plan_and_move[movement]()
            
            sec_passed = rospy.get_time() - start_time 
            
            self.move_feedback = self.set_output_msg(self.move_feedback, success, sec_passed, status)
            self.move_server.publish_feedback(self.move_feedback)
            
            if not success:
                break

        sec_passed = rospy.get_time() - start_time 

        if success:
            status = "action: '{}' of movements: {} succeeded".format(goal.action, goal.movements)
            rospy.loginfo(status)
            self.move_result = self.set_output_msg(self.move_result, success, sec_passed, status)
            self.move_server.set_succeeded(self.move_result)
        else:
            status = "action: '{}' of movements: {} (partly) failed".format(goal.action, goal.movements)
            rospy.logwarn(status)
            self.move_result = self.set_output_msg(self.move_result, success, sec_passed, status)
            self.move_server.set_aborted(self.move_result)
        
    def set_output_msg(self, msg, success, sec_passed, status):
        msg.success = success
        msg.sec_passed = sec_passed
        msg.status = status
        return msg
    
    
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_arm_action_servers")   
    
    actions_base_name = "move_arm" 
    # planner = MoveArmActionServers(actions_base_name)
    planner_id = "RRTConnect"
    num_planning_attempts = 5
    plan_time_sec = 5.0
    
    planner = MoveArmPlanner(planner_id, num_planning_attempts, plan_time_sec, debug=False)
    start_pos = [-0.0, -0.8, 0.0, -2.3, 0.0, 3.1, 0.8] 
    planner.move_group.go(start_pos, wait=True)
    rospy.spin()
    
if __name__ == "__main__":
    main()