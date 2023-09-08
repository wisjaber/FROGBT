#!/usr/bin/env python3

from owlready2 import *        
import os
from rospkg import RosPack
from xmltrials.lxml_script import *
import rospy
import actionlib
from std_msgs.msg import String
from tree_msgs.msg import GeneratebtAction, GeneratebtGoal, RecoveryAction, RecoveryActionGoal,RecoveryActionResult
from lxml import etree
from reset_onto import setpath
# from testy_client import Gbt
from recoverymake import get4recovery, TreeMonitorSubscriber, add_index

onto_path.append(setpath())
onto = get_ontology("btowl3.owl")
onto.load()


def Gbt(xml):
    
    # Create an action client
    client = actionlib.SimpleActionClient('bt_recovery_server', GeneratebtAction)
    
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

class RecoveryBranchManager:
    def __init__(self):
        self.pub = rospy.Publisher('tree_recovery', String, queue_size=10)  # Create a publisher
        self.server = actionlib.SimpleActionServer(
            "recovery_action", RecoveryAction, self.execute, False
        )
        self.server.start()
        self.listen = TreeMonitorSubscriber()

    def find_skill(self,action):
        skills = onto.Skills
        for skill in skills.instances():
            if action in skill.name:
                return skill
            
    def find_goal_condition(self,FC):
        checks_class = onto.Checks
        for individual in checks_class.instances():
            if FC in individual.Subtree[0]:
                return individual

    def execute(self, goal):
        result =RecoveryActionResult()
        rospy.loginfo(f"Received goal, action needs recovery: {goal.action}")
        skill = self.find_skill(goal.action)
        FC,_ = self.listen.get_failed_check()
        # FC= "picked_check" 
        goal_check = self.find_goal_condition(FC)
        recovery_start = etree.fromstring(goal_check.Subtree[0])
        recovery_start = add_index(recovery_start)
        generate_base_xml("bt1")
        rbt = TreeParser("include/trees/bt1.xml")
        rtree = rbt.update_tree()
        btwait = '<initialising_tree name="initialising_tree"/>'
        btwait = etree.fromstring(btwait)
        wait_parent = etree.Element("Fallback", name="recovery_initialization")
        # rbt.MainTree = append_under_selector(rbt.MainTree,recovery_start)
        rec_branch = etree.Element("Fallback", name="recovery_branch")
        rec_branch.append(recovery_start)
        wait_parent.append(btwait)
        wait_parent.append(rec_branch)
        rbt.MainTree.append(wait_parent)
        rbt.write_into_file()
        self.pub.publish("include/trees/bt1.xml")
        self.server.set_aborted()

if __name__ == "__main__":
    rospy.init_node("recovery_node")
    print("recovery action is called...")
    # Create the actions
    recoveryserver = RecoveryBranchManager()
    rospy.loginfo("recovery Server is ready.")
    rospy.spin()
