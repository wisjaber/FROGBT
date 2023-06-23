#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from tree_msgs.srv import LocationCheck, LocationCheckRequest,LocationCheckResponse
from moveit_commander.conversions import pose_to_list

class LocationMonitor():
    def __init__(self):
        rospy.init_node('LocationCheck_server')
        self.LocationCheck_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.location_cb)
        rospy.Service('LocationCheck_Service', LocationCheck, self.handle_location_check)
        rospy.loginfo("Location Check service is ready.")
        # self.current_location = Pose()

    def location_cb(self, msg: PoseWithCovarianceStamped):
        self.current_location = msg.pose

    def handle_location_check(self,req):

        ## orientation z sometimes is positive and sometime negative for the same orientation
        req.LocationCheck.orientation.z=self.current_location.pose.orientation.z
        response = self.all_close(req.LocationCheck,self.current_location.pose,0.4)
        rospy.loginfo("At requested location: {}".format(response))
        return LocationCheckResponse(response)

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
        print("goal",goal)
        print("pose",actual)
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
    location_service = LocationMonitor()
    rospy.spin()
