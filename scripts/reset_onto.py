#!/usr/bin/env python3

import rospy
from rospkg import RosPack
from owlready2 import *  
from tree_msgs.srv import stateCheck, stateCheckResponse
# import onto

def setpath():
    """
    This function sets the path to the tree xml from the relative path of the package
    """
    rp = RosPack()
    package_path = rp.get_path("bt_tests")
    relative = os.path.join("include/ontology/IEEE-1872-2015/")
    path = os.path.join(package_path, relative)
    return path

if __name__ == "__main__":
    # rospy.init_node("ontology_reset_node")
    onto_path.append(setpath())
    onto = get_ontology("btowl.owl")
    onto.load()

    onto.Agent.located[0] = onto.Random
    onto.Object.state[0]= onto.inbox
    onto.Object.located[0]= onto.PickingLocation
    onto.save()

    onto_path.append(setpath())
    onto = get_ontology("btowl2.owl")
    onto.load()

    onto.milk.state[0] = onto.stacked
    onto.milk2.state[0] = onto.stacked
    onto.milk3.state[0] = onto.stacked
    onto.milk4.state[0] = onto.stacked
    onto.save()

    onto_path.append(setpath())
    onto = get_ontology("btowl3.owl")
    onto.load()

    onto.milk.state[0] = onto.stacked
    onto.milk2.state[0] = onto.stacked
    onto.milk3.state[0] = onto.stacked
    onto.milk4.state[0] = onto.stacked
    onto.save()
    
    rospy.loginfo("ontology is reset")
    # rospy.spin()
