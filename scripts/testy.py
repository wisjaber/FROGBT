#!/usr/bin/env python3

from owlready2 import *        
import os
from rospkg import RosPack
from shaka.src.bt_tests.scripts.xmltrials.xml_parser import *
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
    onto_path.append(setpath())
    onto = get_ontology("btowl.owl")
    onto.load()
    # sync_reasoner()
    posc = list(onto.data_properties())
    print(posc)
    # st = onto.search(iri = "*tree", mb=onto.search(is_a=onto.MoveBase))
    # print(st)
    lookup = onto.Skills.instances()
    for skill in lookup:
        if skill.Postcondition == [onto.isAtLocation]:
            print ("found one",skill)
            print(skill.hasChecks)
            for check in skill.hasChecks:
                check_xml=check.Subtree[0]
                file_path = "checkxml.xml"
                create_xml_file(check_xml, file_path)
            subtree_xml=skill.Subtree[0]
            file_path = "subtreexml.xml"
            create_xml_file(subtree_xml, file_path)
            print(skill.Subtree[0])
        else: 
            print("can't find skill")
    
