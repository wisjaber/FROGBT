#!/usr/bin/env python3

# python stuff
import sys
import copy
import rospy
# ros msgs
from geometry_msgs.msg import PoseStamped, Pose
from tree_msgs.srv import HomeCheck, HomeCheckRequest, HomeCheckResponse
import tf

# moveit
from moveit_commander import roscpp_initialize, roscpp_shutdown 
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_commander.conversions import pose_to_list, list_to_pose, list_to_pose_stamped, transform_to_list


class HomeCheckClass():
    def __init__(self):
        rospy.init_node('HomeCheck_server')
        rospy.Service('HomeCheck_service', HomeCheck, self.handle_home_check)
        group_name = "panda_arm" 
        self.move_group = MoveGroupCommander(group_name, wait_for_servers=5)
        self.goal = PoseStamped()
        self.goal.header.frame_id="map"
        self.goal.pose.position.x = -2.5
        self.goal.pose.position.y=-0.5
        self.goal.pose.position.z=1.39
        self.goal.pose.orientation.x=0.69
        self.goal.pose.orientation.y=-0.0
        self.goal.pose.orientation.z=-0.7
        self.goal.pose.orientation.w=0.0

        ####### tf stoef #######
        # self._tl = tf.TransformListener()
        # self._tl.waitForTransform("map", self.goal.header.frame_id, rospy.Time(), rospy.Duration(1.0))  
        # goal_map_frame = self._tl.transformPose("map", self.goal)

    def handle_home_check(self,req):
        actual_pose = self.move_group.get_current_pose()
        tolerance = 0.06
        response =  self.all_close(self.goal,actual_pose, tolerance)
        return HomeCheckResponse(response)
    
    def all_close(self,goal, actual, tolerance):
        # return True
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        assert type(goal) == type(actual)
        # print("goal",goal)
        # print("pose",actual)
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True


if __name__ == "__main__":
    HomeCheck_service = HomeCheckClass()
    rospy.spin()