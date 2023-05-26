#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from tree_msgs.srv import LocationCheck, LocationCheckRequest,LocationCheckResponse
from owlready2 import *      
from rospkg import RosPack

def setpath():
    """
    This function sets the path to the tree xml from the relative path of the package
    """
    rp = RosPack()
    package_path = rp.get_path("bt_tests")
    relative = os.path.join("include/ontology/IEEE-1872-2015/")
    path = os.path.join(package_path, relative)
    return path


class LocationCheck_client():
    def __init__(self):
        rospy.init_node('LocationCheckService_client')
        rospy.wait_for_service('LocationCheck_service')

    def load_owl(self):
        onto_path.append(setpath())
        self.onto = get_ontology("btowl.owl")
        self.onto.load()

    def am_i_there(self):
        self.load_owl()
        try:
            LocationCheckService = rospy.ServiceProxy('LocationCheck_service', LocationCheck)
            request = Pose()
            print("calling")
            request.position.x = self.onto.home.PositionX[0]
            request.position.y = self.onto.home.PositionY[0]
            request.position.z = self.onto.home.PositionZ[0]
            request.orientation.x = self.onto.home.OrientationX[0]
            request.orientation.y = self.onto.home.OrientationY[0]
            request.orientation.z = self.onto.home.OrientationZ[0]
            request.orientation.w = self.onto.home.OrientationW[0]
            print(request)
            response = LocationCheckService(request)
        
            rospy.loginfo("Location Check: {}".format(response))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

if __name__ == "__main__":
    LC = LocationCheck_client()
    LC.load_owl()
    LC.am_i_there()
