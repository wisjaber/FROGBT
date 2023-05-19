#!/usr/bin/env python3

import sys
import rospy
import actionlib
import numpy as np
import tf
import moveit_commander
from copy import deepcopy

from rospkg import RosPack

# ros msgs
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class MoveArmPlanner(object):
  """docstring for PickPlacePlan."""
  def __init__(self, planner_id, num_planning_attempts, plan_time_sec, debug=False):   
    self._num_plan_attempts = num_planning_attempts
    self.move_names = ["home", "retract_arm"]
    self.plan_and_move_funcs = [self.to_home, self.to_retract_arm]
    self.plan_and_move = { name: func for name, func in zip(self.move_names, self.plan_and_move_funcs) }
    

    self.home_config = (self.move_names[0],[-0.0, -0.8, 0.0, -2.3, 0.0, 3.1, 0.8] )

    # self.retract_arm_config = (self.move_names[1], [-2.0491456436606224e-07, 0.040000236805120847,
    #   -1.943671711540027e-07, 0.11836082252256919,-1.1446617794105318, -0.08141332636831855,
    #   -2.507829013831965, -0.2857756781317544, 2.804496977726295, -0.49708356458740965])

    
    # ros stuff
    moveit_commander.roscpp_initialize(sys.argv)
    
    rospy.on_shutdown(self.shutdown_cb)
    self.is_shutdown = False

    self.tf_listener = tf.TransformListener()
    
    timeout=5.0
    self.scene =moveit_commander.PlanningSceneInterface(service_timeout=timeout)
    self.group_name = "panda_arm" # self.move_group.get_group_names()
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name, wait_for_servers=timeout)
    self.robot = moveit_commander.RobotCommander()

    
    self.plan_frame = "base_link" # self.move_group.get_planning_frame()
    self.ee_link = "panda_vacuum" # self.move_group.get_end_effector_link()

    # print(self.move_group.get_active_joints())
    # print(self.move_group.get_current_joint_values())

    # planner settings
    self.move_group.set_planner_id(planner_id)
    self.move_group.set_num_planning_attempts(num_planning_attempts)
    self.move_group.set_planning_time(plan_time_sec)
    
        
    self.status = "MoveArmPlanner class Initialized"
    rospy.loginfo(self.status)
    
  def set_planner_id(self, planner_id):
    success = False
    status = ""   
    self.move_group.set_planner_id(planner_id)
    settings = self.get_planner_settings()
    if settings.id == planner_id:
      success = True
      status = "planning attempts applied succesfully"
    else:
      status = "planning attempts not applied"
    rospy.loginfo(status)
    
    return success, status
    
  def set_num_planning_attempts(self, plan_attemempts): 
    success = False
    status = ""   
    self.move_group.set_num_planning_attempts(plan_attemempts)
    self._num_plan_attempts = plan_attemempts
    settings = self.get_planner_settings()
    if settings.attempts == plan_attemempts:
      success = True
      status = "planning attempts applied succesfully"
    else:
      status = "planning attempts not applied"
    rospy.loginfo(status)
    return success, status
    
  def set_planning_time(self, plan_time_sec):
    success = False
    status = ""
    self.move_group.set_planning_time(plan_time_sec)
    if self.move_group.get_planning_time() == plan_time_sec:
      status = "planning time successfully set"
      success = True
    else:
      status = "planning time not set!"
    rospy.loginfo(status)
    return success, status

    
  def to_home(self):
    success, status = self.to_joint_config_jointspace(self.home_config[1], self.home_config[0])
    return success, status
    
  def to_retract_arm(self):
    success, status = self.to_joint_config_jointspace(self.retract_arm_config[1], self.retract_arm_config[0])

    return success, status

  def to_pose_goal_cartspace(self, goal_list , name, bound=0.015, step=0.01):
    success = False
    status = ""
    
    goals = []
    for goal in goal_list:
      if isinstance(goal, PoseStamped):
        goal = goal.pose
      elif isinstance(goal,list):
        goal = moveit_commander.conversions.list_to_pose(goal)
      elif not isinstance(goal, Pose):
        status = "goal input is of type {} and should be of type list, pose or pose stamped".format(type(goal)) 
        rospy.loginfo(status)
        return success, status
      goals.append(goal)
    
    plan, success, status = self.get_cartesian_plan(goals, name, step)
    if not success:
      return success, status
    
    success, status = self.execute_plan(plan, name)
    if not success:
      return success, status
    
    success = all_close(goals[-1], 
                      self.move_group.get_current_pose().pose, bound)

    if success:
      status = "movement {} succeeded within {} bound".format(name, bound) 
    else:
      status = "movement {} did not succeed within {} bound".format(name, bound)      
      
    return success, status
    
  def to_pose_goal_jointspace(self, goal, name, bound=0.015):
    success = False
    status = ""
    
    if isinstance(goal, PoseStamped):
      goal = goal.pose
    elif isinstance(goal,list):
      goal = moveit_commander.conversions.list_to_pose(goal)
    elif not isinstance(goal, Pose):
      status = "goal input is of type {} and should be of type list, pose or pose stamped".format(type(goal))
      rospy.loginfo(status)
      return success, status
    
    plan, success, status = self.get_plan(goal, name)
    if not success:
      return success, status
    
    success, status = self.execute_plan(plan, name)
    if not success:
      return success, status
    
    success = all_close(goal, self.move_group.get_current_pose().pose, bound)

    if success:
      status = "movement {} succeeded within {} bound".format(name, bound) 
    else:
      status = "movement {} did not succeed within {} bound".format(name, bound) 
    
    return success, status
  
  def to_joint_config_jointspace(self, joint_config, name, bound=0.05):
    success = False
    status = ""
    
    if not isinstance(joint_config, list):
      status = "joint_config input is of type {} and should be of type list()".format(type(joint_config))
      rospy.loginfo(status)
      return success, status
    
    plan, success, status = self.get_plan(joint_config, name)
    if not success:
      return success, status
    
    success, status = self.execute_plan(plan, name)
    if not success:
      return success, status
    
    success = all_close(joint_config, 
                      self.move_group.get_current_joint_values(), bound)

    if success:
      status = "movement {} succeeded within {} bound".format(name, bound) 
    else:
      status = "movement {} did not succeed within {} bound".format(name, bound) 
    
    return success, status
        
  def execute_plan(self, plan, name):
    # might wait for execution and check motion in rviz:
    # check rviz MotionPlanning -> Planned Path with topic:  /move_group/display_planned_path
    success = False
    status = ""

    if self.debug:
      result, status = self.term_input("press y: to execute or q: to quit and set to failure: {}".format(name))
    else:
      success = True
      status = "Not debug"
      result = 'y'
      # return success, status

    if result == 'q':
      return success, status
    
    if result == 'r':
      return success, status
      # get current goal
      # TODO: allow replanning
      
    
    
      
    if not self.is_shutdown and result == 'y':
      rospy.loginfo('started moveing')
      self.move_group.execute(plan, wait=True)
      rospy.loginfo('stopped moveing')
      
      self.move_group.stop()
      self.move_group.clear_pose_targets() 

      
      success = True
      status = "movement made for: {}".format(name)

    else:
      status = "No (complete) movement made for: {}. Shutdown requested during execute".format(name)

    return success, status
  
  def get_cartesian_plan(self, pose_msg_list, name, step=0.01):
    rospy.loginfo("trying to get plan for {}".format(name))
    waypoints = []
    for pose_msg in pose_msg_list:
      waypoints.append(deepcopy(pose_msg))
    
    (plan, fraction) = self.move_group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      step,        # eef_step
                                      0.00)         # jump_threshold
    
    success, status = self.plan_is_found(plan, name)
    if not success:
      status = status + str(", completed fraction of traj is: {}".format(fraction))
      
    if fraction < 0.99:
      status = "did not complete full path fraction: {}".format(fraction)
      success = False
      rospy.logwarn(status)
    
    return plan, success, status
    
  def get_plan(self, goal, name):
    # Todo: check if input is valid
    '''input joint list, pose msg or JointState msgs, ouput plan (RobotTrajectory msg)'''
    rospy.loginfo("trying to get plan for {}".format(name))
    
    plan = self.move_group.plan(goal)
    
    success, status = self.plan_is_found(plan, name)

    return plan, success, status

  def plan_is_found(self, plan, name):
    success = False
    status = ""
    
    if len(plan.joint_trajectory.points) == 0:
      status = "No plan could be made for: {}".format(name) 
    else:
      status = "plan found for: {}".format(name)
      success = True
    
    rospy.loginfo(status)
      
    return success, status
  
  def term_input(self, sentence):
    res = ""
    status = "No input given"
    try:
      res = raw_input(sentence)
      if res == 'y':
        status = "Plan accepted"
        
      elif res == 'r':
        status = "replanning"
    
      elif res == 'q':
        status = "Plan rejected and returns failed"
        
      else:
        print("select either y: yes, r: replane or q: to quit and set to failure")
        self.term_input(self, sentence)
      
    except (KeyboardInterrupt, ValueError):
      # exit()
      pass     
    return res, status
    
  def shutdown_cb(self):
    rospy.loginfo("Shutdown requested")
    self.is_shutdown = True

      
def all_close(goal, actual, tolerance):
  # return True
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  assert type(goal) == type(actual)

  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is Pose:
    return all_close(moveit_commander.conversions.pose_to_list(goal), moveit_commander.conversions.pose_to_list(actual), tolerance)

  return True

  
def main():
    rospy.init_node("move_arm_planner", anonymous=True)

    
    # init pickPlacePlanner class
    planner_id = "RRTConnect"
    num_planning_attempts = 5
    plan_time_sec = 10.0
    planner = MoveArmPlanner(planner_id, num_planning_attempts, plan_time_sec ,debug=False)


    planner.to_home()
    
    # move to start position
    planner.to_retract_arm()
      
    rospy.loginfo("finished")
  
  
  
if __name__ == '__main__':
    main()

    