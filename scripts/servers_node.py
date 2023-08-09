#!/usr/bin/env python3

import rospy
import actionlib
import time
from rospkg import RosPack
from owlready2 import *  
from tree_msgs.srv import stateCheck, stateCheckResponse
from tree_msgs.msg import MockAction, MockGoal, MockResult


def setpath():
    """
    This function sets the path to the tree xml from the relative path of the package
    """
    rp = RosPack()
    package_path = rp.get_path("bt_tests")
    relative = os.path.join("include/ontology/IEEE-1872-2015/")
    path = os.path.join(package_path, relative)
    return path

onto_path.append(setpath())
onto = get_ontology("btowl.owl")
onto.load()

class MockDriveServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "Drive_fake", MockAction, self.execute, False
        )
        self.server.start()

    def find_indi(self,g):
        for goal_instance in onto.Location.instances():
            if g in goal_instance.name:
                print(goal_instance.name)
                return goal_instance
            
    def execute(self, goal):
        rospy.loginfo(f"Received goal: {goal.location}")

        # Do any processing or actions with the goal string here.
        # For this example, we'll simply set the Location property
        # to the received goal string.
        time.sleep(1)  # Sleep for 1 second
        
        indi = self.find_indi(goal.location)
        if indi is not None:
            print(indi,onto.Agent.located)
            onto.Agent.located[0] = indi
            result = MockResult()
            onto.save()
            print(onto.Agent.located)
            result.success = True

        rospy.loginfo(f"Location set to: {goal.location}")
        self.server.set_succeeded(result)

class MockPickServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "Pick_fake", MockAction, self.execute, False
        )
        self.server.start()
            
    def execute(self, goal):
        rospy.loginfo(f"Received goal: {goal.location}")

        time.sleep(1)  # Sleep for 1 second
        onto.Object.state[0]= onto.picked
        result = MockResult()
        # onto.save()
        result.success = True
        onto.save()
        print(onto.Object.state)
        rospy.loginfo(f"Object has been {goal.location}")
        self.server.set_succeeded(result)

class MockPlaceServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "Place_fake", MockAction, self.execute, False
        )
        self.server.start()
            
    def execute(self, goal):
        rospy.loginfo(f"Received goal: {goal.location}")

        # Do any processing or actions with the goal string here.
        # For this example, we'll simply set the Location property
        # to the received goal string.
        time.sleep(1)  # Sleep for 1 second
        onto.Object.state[0]= onto.placed
        result = MockResult()
        onto.save()
        print(onto.Object.state)
        onto.save()
        result.success = True

        rospy.loginfo(f"Object has been {goal.location}")
        self.server.set_succeeded(result)

class MockDetectServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "Detect_fake", MockAction, self.execute, False
        )
        self.server.start()
            
    def execute(self, goal):
        rospy.loginfo(f"Received goal: {goal.location}")

        # Do any processing or actions with the goal string here.
        # For this example, we'll simply set the Location property
        # to the received goal string.
        time.sleep(1)  # Sleep for 1 second
        onto.Agent.located[0]= onto.Object.located[0]
        onto.Object.state[0]= onto.detected
        result = MockResult()
        onto.save()
        print(onto.Object.state)
        result.success = True

        rospy.loginfo(f"Object has been {goal.location}")
        self.server.set_succeeded(result)

def stateCheck_callback(request):
    # sync_reasoner()
    response = stateCheckResponse()
    if request.state in onto.Object.state[0].name:
        print(onto.Object.state[0].name, request.state)
        response.response = True
        return response
    else:
        print(onto.Object.state[0].name)
        response.response = False
        return response

def locationCheck_callback(request):
    # sync_reasoner()
    response = stateCheckResponse()
    if request.state in onto.Agent.located[0].name:
        print(onto.Agent.located[0].name, request.state)
        response.response = True
        return response
    else:
        print("WHAT",onto.Agent.located[0].name,request.state)
        response.response = False
        return response
    
if __name__ == "__main__":
    rospy.init_node("servers_node")
    print("ontology loaded...")

    # Create the services
    stateCheckService = rospy.Service("stateCheck", stateCheck, stateCheck_callback)
    rospy.loginfo("stateCheck Server is ready.")
    locationCheckService = rospy.Service("locationCheck", stateCheck, locationCheck_callback)
    rospy.loginfo("locationCheck Server is ready.")

    # Create the actions
    pickserver = MockPickServer()
    rospy.loginfo("Pick_fake Server is ready.")
    driveserver = MockDriveServer()
    rospy.loginfo("Drive_fake Server is ready.")
    placeserver = MockPlaceServer()
    rospy.loginfo("Place_fake Server is ready.")
    detectserver = MockDetectServer()
    rospy.loginfo("Detect_fake Server is ready.")

    rospy.spin()