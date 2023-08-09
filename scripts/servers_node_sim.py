#!/usr/bin/env python3

import rospy
import actionlib
import time
from rospkg import RosPack
from owlready2 import *  
from tree_msgs.srv import product_stateCheck, product_stateCheckResponse
# from tree_msgs.msg import MockAction, MockGoal, MockResult
from tree_msgs.srv import VacuumCheck 
# from apriltag_filter.srv import FilterAprilTagDetections, FilterAprilTagDetectionsResponse, FilterAprilTagDetectionsRequest
from albert_skills.msg import PickAction,PickActionGoal,PickActionResult, PlaceAction, PlaceActionResult, PlaceGoal
from testy_client import setpath

onto_path.append(setpath())
onto = get_ontology("btowl2.owl")
onto.load()

def get_indi(tag):
    try:
        for product in onto.Product.instances():
            if product.tag[0]==str(tag):
                return product
    except:
        print("product couldn't be found")

def isvacuum():
    vacuum_check = rospy.ServiceProxy('VacuumCheckService', VacuumCheck)
    vacuumresponce = vacuum_check()
    # print(vacuumresponce.response)
    return vacuumresponce.response

# def isdetect(tag):
#     #  if( rep.filtered_pose.position.x != 0)
#     detectcheck = rospy.ServiceProxy('filter_apriltag_detection', FilterAprilTagDetections)
#     req = FilterAprilTagDetectionsRequest()
#     req.tag_id = int(tag)
#     detectresponce = detectcheck(req)
#     if detectresponce.filtered_pose.position.x != 0:
#         return True
#     else:
#         return False

class MockPickServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "pickstate_server", PickAction, self.execute, False
        )
        self.client = actionlib.SimpleActionClient('pick_server', PickAction)

        self.server.start()
            
    def execute(self, goal):
        # result = PickActionResult()
        rospy.loginfo(f"Received goal: {goal.goal_id}")
        # Forward the received goal to the other action server
        self.client.wait_for_server()
        self.client.send_goal(goal)
        # Wait for the result from the other action server
        self.client.wait_for_result()
        result = self.client.get_result()

        # goal = PickActionGoal(goal_id=goal.goal_id)
        # result = self.client.send_goal(goal)
        # Waits for the server to finish performing the action.
        # self.client.wait_for_result(100)
        # res = self.client.get_state()
        print("RESULT",result)
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            product = get_indi(goal.goal_id)
            product.state[0]=onto.picked
            result = True
            onto.save()
            rospy.loginfo(f"Object has been picked")
            self.server.set_succeeded()
        else:
            self.server.set_aborted()
            rospy.loginfo("Action failed")
            rospy.loginfo(f"Result: {result}")
        # self.server.set_succeeded(result.success)
        
class MockPlaceServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "placestate_server", PickAction, self.execute, False
        )
        self.client = actionlib.SimpleActionClient('place_server', PickAction)

        self.server.start()
            
    def execute(self, goal):
        # result = PickActionResult()
        rospy.loginfo(f"Received goal: {goal.goal_id}")
        # Forward the received goal to the other action server
        self.client.wait_for_server()
        self.client.send_goal(goal)
        # Wait for the result from the other action server
        self.client.wait_for_result()
        result = self.client.get_result()

        # goal = PickActionGoal(goal_id=goal.goal_id)
        # result = self.client.send_goal(goal)
        # Waits for the server to finish performing the action.
        # self.client.wait_for_result(100)
        # res = self.client.get_state()
        print("RESULT",result)
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            product = get_indi(goal.goal_id)
            product.state[0]=onto.placed
            result = True
            onto.save()
            rospy.loginfo(f"Object has been placed")
            self.server.set_succeeded()
        else:
            self.server.set_aborted()
            rospy.loginfo("Action failed")
            rospy.loginfo(f"Result: {result}")
        # self.server.set_succeeded(result.success)


def pick_stateCheck_callback(request):
    response = product_stateCheckResponse()
    product = get_indi(request.tag)
    print(request.state,product.state[0]) #
    gripper = isvacuum()
    if (request.state in product.state[0].name) and gripper:
        print(product.state[0].name, request.state) #
        response.response = True
        return response
    else:
        print(product.state[0].name) #
        response.response = False
        return response
    
def place_stateCheck_callback(request):
    response = product_stateCheckResponse()
    product = get_indi(request.tag)
    gripper = isvacuum()
    if request.state in product.state[0].name and not gripper:
        print(product.state[0].name, request.state) #
        response.response = True
        return response
    else:
        print(product.state[0].name)  ####
        response.response = False
        return response


if __name__ == "__main__":
    rospy.init_node("servers_node")
    print("ontology loaded...")

    # Create the services
    pick_stateCheckService = rospy.Service("pick_stateCheck", product_stateCheck, pick_stateCheck_callback)
    rospy.loginfo("pick_stateCheck Server is ready.")
    place_stateCheckService = rospy.Service("place_stateCheck", product_stateCheck, place_stateCheck_callback)
    rospy.loginfo("pick_stateCheck Server is ready.")

    # Create the actions
    pickserver = MockPickServer()
    rospy.loginfo("PickAction Server is ready.")
    placeserver = MockPlaceServer()
    rospy.loginfo("PlaceAction Server is ready.")


    rospy.spin()