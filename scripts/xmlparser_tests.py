#!/usr/bin/env python3

from owlready2 import *        
import os
from rospkg import RosPack
from xmltrials.lxml_script import *
import rospy
import actionlib
from std_msgs.msg import String
from tree_msgs.msg import GeneratebtAction, GeneratebtGoal
from lxml import etree

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
onto = get_ontology("btowl2.owl")
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
    goal_instances = []
    for goal_instance in onto.individuals():
        if goal in goal_instance.name:
            goal_instances.append(goal_instance)
    if len(goal_instances)==1:
        return goal_instances[0]
    return goal_instances

def get_properties(goal, with_indi = False):
    if not with_indi:
        indi = find_individual(goal)
    else:
        indi = goal
    indi_properties = indi.get_properties()
    return indi_properties

def get_data_property(goal,with_indi = False):
    indi_properties = get_properties(goal,with_indi)
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

def update_goal_location(sortedprop,goal):
    try: 
        goal= goal[0]
        print("goal",goal[0])
        print("sortef",sortedprop)
    except:
        print("goal is not a list",goal)
        print("sortef",sortedprop)
    loc_values = []
    for prop in sortedprop:
        value = getattr(goal,prop.python_name)
        loc_values.append(str(value[0]))
    goal_value = ";".join(loc_values)
    return goal_value

    # # for child in sbt.iter("Action"):
    # for child in sbt.iter():
    #     if "goal" in child.attrib:
    #         child.attrib["goal"]=goal_value
    # return sbt

def update_goal_tag(tag,sbt):
    # for child in sbt.iter("Action"):
    for child in sbt.iter():
        if "tag" in child.attrib:
            child.attrib["tag"]=tag
    return sbt

def which_actions(actions,requested_action):
    for action in actions:
        if requested_action in action[0].name:
            return action[0]
        
def which_postchecks(action):
    instance = action.Postcondition[0]
    return instance.hasChecks
    
def find_checks_and_actions(goal_instance):
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

def get_goal_details(goal, details=None):
    goal_instance = find_individual(goal)
    try: 
        if len(goal_instance)>1:
            if details is not None:
                for goal_indi in goal_instance:
                    if goal_indi.tag[0] == details:
                        goal_instance = goal_indi
                        tag = goal_instance.tag[0]
                        break
            else:
                goal_instance = goal_instance[0]
                tag = goal_instance.tag[0]

            gdp = get_data_property(goal_instance,with_indi=True)
            
    except:
        gdp = get_data_property(goal_instance,True)
        if goal_instance.tag:
            tag = goal_instance.tag[0]


    try: 
        sorted = sort_dataproperty_destination(gdp)
        return goal_instance, 0,(goal_instance,sorted)
    except: 
        goal_location = goal_instance.located
        gdplocation = get_data_property(goal_location[0],with_indi=True)
        sorted_location = sort_dataproperty_destination(gdplocation)

        # print("gdp is not a location",sorted_location)
        return goal_instance, tag, (goal_location,sorted_location)

def get_action_for_condition(condition):
    checks_class = onto.Checks
    for individual in checks_class.instances():
        if condition in individual.Subtree[0]:
            print("individual found ", individual.name)
            postcondtion_instance = individual.isCheckFor[0]
            actions_with_postcondtion = postcondtion_instance.Effect
            print(actions_with_postcondtion)
            return actions_with_postcondtion

def create_atomicBT(root, actions):
    action_seq = []
    if len(actions)>1:
        atomic_selector = etree.Element("Selector", name="multiple_actions_Fallback")
    for act in actions:
        if act.Precondition:
            precon = act.Precondition
            for con in precon:
                action_seq.append(con.hasChecks[0].Subtree[0])

        if act.Treemodel:
            root.append(etree.fromstring(act.Treemodel[0]))
        try: 
            action_seq.append(act.Subtree[0])

        except:
             rospy.logerr("Action %s has no subtree to satisfy it!",str(act.name))
        atomic_sequence = etree.Element("Sequence", name=act.name+"_sequence")
        for string in action_seq:

            atomic_sequence.append(etree.fromstring(string))
        try: 
            atomic_selector.append(atomic_sequence)
        except:
            pass
    try:
        return atomic_selector
    except:
        return atomic_sequence

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
    
def parse_goal(action_required,goal_direct,goal_details):

    goal_instance,tag ,location = get_goal_details(goal_direct,goal_details)
    print("##############",tag)
    checks, actions = find_checks_and_actions(goal_instance)
    
    related_action = which_actions(actions,action_required)
    try:
        related_checks = which_postchecks(related_action)
        return related_checks,related_action,tag, location
    except:
        return checks[0],actions[0],tag,location

if __name__ == "__main__":
    rospy.init_node('Gbt_node')
    if len(sys.argv) > 1:
        try:
            action_required = sys.argv[1]
            goal_direct = sys.argv[2]
            goal_extras = sys.argv[3]
        except:
            goal_extras = None

    # goal_destination = "waypoint1"
    # result = False
    # action_required = "go"
    # goal_direct = "home"
    # action_required = "pick"
    # goal_direct = "milk"
    # goal_extras = None
    # goal_extras = "21"

    ### the following lines generate the tree
    generate_base_xml()

    BTparsed = TreeParser("include/trees/bt.xml")
    tree = BTparsed.update_tree()
    BTparsed.add_groot_node()
    result = Gbt("include/trees/bt.xml")
    # BTparsed.remove_groot_node()

    ChecksMonitor = TreeMonitorSubscriber()
    checks,actions,goal_tag,goal_location = parse_goal(action_required,goal_direct,goal_extras)
    # # print(checks,actions)
    
    for check in checks:
        print(check)
        checktree = check.Subtree[0]
        checktree = etree.fromstring(checktree)
        checktree = update_element_values(checktree,update_goal_location(goal_location[1],goal_location[0]), goal_tag)
        # BTparsed.MainTree[0].append(checktree)
        append_under_selector(BTparsed.MainTree[0],checktree)
        BTparsed.write_into_file()

        goal_individual = find_individual(goal_direct)
        find_info_about_goal(goal_individual)
    result = Gbt("include/trees/bt.xml")
    try:
        while not result and not rospy.is_shutdown():
            rospy.loginfo('generated bt failed, recalculating')
            failed_check = ChecksMonitor.get_failed_check()
            if failed_check:
                try:
                    print("Latest message received:", failed_check)
                    action_individuals = get_action_for_condition(failed_check)
                    atomicBT = create_atomicBT(BTparsed.root,action_individuals)
                    atomicBT = update_element_values(atomicBT,update_goal_location(goal_location[1],goal_location[0]), goal_tag)
                    BTparsed.insert_atomicBT(failed_check,atomicBT)
                    result = Gbt("include/trees/bt.xml")
                except:
                    print("Skill knowledgebase is lacking")
                    
            else:
                print("ROS Exception: failed check wasn't found")
                break
        if result:
            print("Tree was a success!")
        else:
            print("an exception flag was up and bt failed to generate")

    except rospy.ROSInterruptException:
        pass


        # action_individuals = get_action_for_condition("vacuum_check")
        # atomicBT = create_atomicBT(BTparsed.root,action_individuals)
        # atomicBT = update_element_values(atomicBT,update_goal_location(goal_location[1],goal_location[0]), goal_tag)
        # BTparsed.insert_atomicBT("vacuum_check",atomicBT)

        # action_individuals = get_action_for_condition("tag_detection_check")
        # atomicBT = create_atomicBT(BTparsed.root,action_individuals)
        # atomicBT = update_element_values(atomicBT,update_goal_location(goal_location[1],goal_location[0]), goal_tag)
        # BTparsed.insert_atomicBT("tag_detection_check",atomicBT)

        # action_individuals = get_action_for_condition("location_check")
        # atomicBT = create_atomicBT(BTparsed.root,action_individuals)
        # atomicBT = update_element_values(atomicBT,update_goal_location(goal_location[1],goal_location[0]), goal_tag)
        # BTparsed.insert_atomicBT("location_check",atomicBT)



   

