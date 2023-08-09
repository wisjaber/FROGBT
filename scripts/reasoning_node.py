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
from testy_client import Gbt

onto_path.append(setpath())
onto = get_ontology("btowl2.owl")
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

    def find_skill(self,action):
        skills = onto.Skills
        for skill in skills.instances():
            if action in skill.name:
                return skill
            
    def execute(self, goal):
        result =RecoveryActionResult()
        
        rospy.loginfo(f"Received goal, action needs recovery: {goal.action}")
        skill = self.find_skill(goal.action)
        act_fb = skill.hasFallback
        recovery_branch = etree.fromstring(act_fb[0].Subtree[0])
        generate_base_xml("bt1")
        BTparsed = TreeParser("include/trees/bt.xml")
        rbt = TreeParser("include/trees/bt1.xml")
        tree = BTparsed.update_tree()
        rtree = rbt.update_tree()
        action_element, action_child,action_parent = failed_action(goal.action,goal.index,tree)
        btwait = '<initialising_tree name="initialising_tree"/>'
        btwait = etree.fromstring(btwait)
        wait_parent = etree.Element("Fallback", name="recovery_initialization")
        sequence = etree.Element("Sequence", name="recovery_sequence")
        sequence.append(recovery_branch)
        # recoverynode = action_element.pop
        # action_element.remove(recoverynode)
        # sequence.append(action_element)
        # action_parent.replace(action_child,sequence)
        wait_parent.append(btwait)
        wait_parent.append(sequence)
        rbt.MainTree.append(wait_parent)
        rbt.write_into_file()
        self.pub.publish("include/trees/bt1.xml")
        self.server.set_aborted()

        # result = Gbt("include/trees/bt1.xml")
        # if result:
        #     rospy.loginfo("RECOVERY BRANCH HAS BEEN ADDED?")
        #     self.server.set_succeeded()
        # else:
        #     self.server.set_aborted()

if __name__ == "__main__":
    rospy.init_node("recovery_node")
    print("recovery action is called...")
    # Create the actions
    recoveryserver = RecoveryBranchManager()
    rospy.loginfo("recovery Server is ready.")
    rospy.spin()
