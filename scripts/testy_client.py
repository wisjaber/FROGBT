#!/usr/bin/env python3

from owlready2 import *        
import os
from rospkg import RosPack
from xmltrials.xml_trials import *
import rospy
import actionlib
from std_msgs.msg import String
from tree_msgs.msg import GeneratebtAction, GeneratebtGoal


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
# sync_reasoner()


def Gbt(xml):
    
    # Create an action client
    client = actionlib.SimpleActionClient('bt_generation_server', GeneratebtAction)
    
    # Wait for the action server to start
    client.wait_for_server()
    
    # Create a goal message
    goal = GeneratebtGoal()
    goal.xml = xml
    
    # Send the goal to the action server
    client.send_goal(goal)
    
    # Wait for the result
    client.wait_for_result()
    
    # Get the result
    result = client.get_result()
    return result.success
    # Process the result
    if result.success:
        rospy.loginfo('BT generation successful in client')
    else:

        rospy.loginfo('BT generation failed in client')

def find_info_about_goal(goal_instance):
    if goal_instance.__class__ == onto.Product:
        print(goal_instance.tag)
        print(goal_instance.located)
        location = find_info_about_goal(goal_instance.located)
    elif goal_instance.__class__ == onto.Location:
        goal_location_dp = get_data_property(goal_direct)
        sort_goal_location = sort_dataproperty_destination(goal_location_dp)
    # return goal_instance

def find_individual(goal):
    for goal_instance in onto.individuals():
        if goal in goal_instance.name:
            return goal_instance

def get_properties(goal):
    indi = find_individual(goal)
    indi_properties = indi.get_properties()
    return indi_properties

def get_data_property(goal):
    indi_properties = get_properties(goal)
    data_properties = [prop for prop in indi_properties if isinstance(prop, DataPropertyClass)]
    return data_properties

def sort_dataproperty_destination(dataproperty):
    property_order = [
    "PositionX",
    "PositionY",
    "PositionZ",
    "OrientationX",
    "OrientationY",
    "OrientationZ",
    "OrientationW"]
    sortedprop = sorted(dataproperty, key=lambda prop: property_order.index(prop.name))
    return sortedprop

def update_goal_location(sortedprop,goal,sbt):
    loc_values = []
    for prop in sortedprop:
        value = getattr(goal,prop.python_name)
        loc_values.append(str(value[0]))
    goal_value = ";".join(loc_values)

    # for child in sbt.iter("Action"):
    for child in sbt.iter():
        if "goal" in child.attrib:
            child.attrib["goal"]=goal_value
    return sbt

def which_actions(actions,requested_action):
    for action in actions:
        if requested_action in action[0].name:
            return action[0]
        
def which_postchecks(action):
    instance = action.Postcondition[0]
    return instance.hasChecks
    
def find_checks_and_actions(goal):
    goal_instance = find_individual(goal)
    class_name = goal_instance.__class__
    class_individuals = class_name.instances()
    actions_with_effect = []
    checks_for_effect = []
    for individual in class_individuals:
        for individual_prop in individual.get_properties():
            if "Effect" in individual_prop.name:
                # print(f"Instance Name: {individual}")
                actions=individual.Effect
                checks=individual.hasChecks
                actions_with_effect.append(actions)
                checks_for_effect.append(checks)
    return checks_for_effect,actions_with_effect

def go_tree_generation(goal):
    checks,actions=find_checks_and_actions(goal)
    return checks,actions

def get_action_for_condition(condition):
    checks_class = onto.Checks
    for individual in checks_class.instances():
        if condition in individual.Subtree[0]:
            print("individual found ", individual.name)
            postcondtion_instance = individual.isCheckFor[0]
            actions_with_postcondtion = postcondtion_instance.Effect
            print(actions_with_postcondtion)
            return actions_with_postcondtion

def create_atomicBT(actions):
    action_seq = []
    for act in actions:
        if act.Precondition:
            precon = act.Precondition
            print(precon)
            for con in precon:
                action_seq.append(con.hasChecks[0].Subtree[0])
        try: 
            action_seq.append(act.Subtree[0])
        except:
             rospy.logerr("Action %s has no subtree to satisfy it!",str(act.name))
        actiontree = g_action_seq(action_seq)
        return actiontree

class TreeMonitorSubscriber:
    def __init__(self):
        rospy.Subscriber("/tree_monitor", String, self.callback)
        self.latest_message = None

    def callback(self, data):
        # Callback function to handle received messages
        self.latest_message = data.data

    def get_failed_check(self):
         # Get the latest received message
        return self.latest_message
    
def parse_goal(action_required,goal_direct):
    if action_required=="go":
        checks, actions = go_tree_generation(goal_direct)
        return checks,actions
    elif action_required=="pick":
        checks, actions = find_checks_and_actions(goal_direct)
        related_action = which_actions(actions,action_required)
        related_checks = which_postchecks(related_action)
        return related_checks,related_action

if __name__ == "__main__":
    # rospy.init_node('Gbt_node')
    # if len(sys.argv) > 1:
    #     action_required = sys.argv[1]
    #     goal_direct = sys.argv[2]
        # goal_direct = sys.argv[1]
    # goal_destination = "waypoint1"
    result = False
    action_required = "pick"
    goal_direct = "milk"

    ### the following lines generate the tree
    generate_base_xml()
    # btwait = '<initialising_tree name="initialising_tree"/>'
    # btwait = create_xml_from_string(btwait)
    # add_under_FB("input.xml",btwait,"BehaviorTree","input.xml")

    ChecksMonitor = TreeMonitorSubscriber()

    checks,actions = parse_goal(action_required,goal_direct)
    print(checks,actions)
    # checks, actions = go_tree_generation(goal_direct)

    for check in checks:
        checktree = check.Subtree[0]
        checktree = create_xml_from_string(checktree)
        # add_to_parent("input.xml",checktree,"BehaviorTree","include/bt_tests/input.xml")
        # appendit("input.xml",checktree,"include/bt_tests/input.xml")
        add_under_FB("input.xml",checktree,"BehaviorTree","BT","include/bt_tests/input.xml")

        goal_individual = find_individual(goal_direct)
        find_info_about_goal(goal_individual)
    # result = Gbt("include/bt_tests/input.xml")
    # while not result:
    if not result:
        rospy.loginfo('generated bt failed, recalculating')
        # failed_check = ChecksMonitor.get_failed_check()
        # if failed_check:
        #     print("Latest message received:", failed_check)
        #     action_individuals = get_action_for_condition(failed_check)
        # else:
        #     print("ROS Exception: failed check wasn't found")
        action_individuals = get_action_for_condition("grasped_check")
        atomicBT = create_atomicBT(action_individuals)
        # update_child_position("include/bt_tests/input.xml","grasped_check","Fallback","grasped_check_fallback")
        update_child_position("include/bt_tests/input.xml", "BT_fallback", "Fallback","grasped_check_fallback")

        # c = find_within_xml("include/bt_tests/input.xml","name","grasped_check")
        # print("found child",printxml(c))
        # add_to_parent("include/bt_tests/input.xml",atomicBT,"Fallback","grasped_check_fallback","include/bt_tests/input.xml")

        # action_individuals = get_action_for_condition("filter_apriltag_detection")
        # atomicBT = create_atomicBT(action_individuals)
        # add_to_parent("include/bt_tests/input.xml",atomicBT,"Fallback",("filter_apriltag_detection"+"_fallback"),"include/bt_tests/input.xml")






    #     add_under_FB("input.xml",checktree,"BehaviorTree","include/bt_tests/input.xml")
    #     # add_to_parent("input.xml",checktree,"Fallback","include/bt_tests/input.xml")
    #     result = Gbt("include/bt_tests/input.xml")
    #     if result:
    #         rospy.loginfo('generated bt was successful')
    #     if not result:
    #         rospy.loginfo('generated bt failed, recalculating')
    #         failed_check = ChecksMonitor.get_failed_check()
    #         if failed_check:
    #             print("Latest message received:", failed_check)
    #             action_individuals = get_action_for_condition(failed_check)
    #         else:
    #             print("ROS Exception: failed check wasn't found")
    #         atomicBT = create_atomicBT(action_individuals)
    #         actiontree = update_goal_location(sort_goal_location,goal_individual,atomicBT)
    #         # add_to_parent("input.xml",actiontree,"Fallback","input.xml")
    #         add_to_parent("include/bt_tests/input.xml",actiontree,"Fallback","include/bt_tests/input.xml")
    #         result=Gbt("include/bt_tests/input.xml")
    #         if result: 
    #             rospy.loginfo('generated bt was successful')
    #         if not result: 
    #                 rospy.loginfo('big nope')

    # ## uncomment the following lines to try the full previously generated tree
    # # result = Gbt("include/bt_tests/input.xml")
    # # if result:
    # #     rospy.loginfo('generated bt was successful')
    # # if not result:
    # #     rospy.loginfo('generated bt failed, recalculating')


