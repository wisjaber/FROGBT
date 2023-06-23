from lxml import etree
from copy import deepcopy

def generate_base_xml():
    # Create the root element
    root = etree.Element("root", main_tree_to_execute="BehaviorTree")
    BehaviorTree = etree.SubElement(root,"BehaviorTree", ID="BehaviorTree")
    tree = etree.ElementTree(root)
    tree.write("include/trees/bt.xml", pretty_print=True)

def printtree(elementtree):
    print(etree.tostring(elementtree))

def insert_to_parent(parent,index,element):
    parent.insert(index,element)
    return parent

def has_children(element):
    if len(element):
        return True

def move_under_parent(parent,old_index,new_index):
    parent[new_index] = parent[old_index]
    return parent

def get_attribute_from_element(element,attribute):
    return element.get(attribute)

def set_attribute_in_element(element,attribute,value):
    element.set(attribute,value)
    return element

def find_child_with_attribute(tree,attribute,value):
    for elem in tree.iter():
        for child in elem:
            if child.get(attribute) == value:
                return child
            
def find_child_with_tag(tree,tag):
    for elem in tree.iter():
        for child in elem:
            if tag in child.tag:
            # if child.tag == tag:
                return child

def get_child_index(parent, child):
    for i, elem in enumerate(parent.iterchildren()):
        if elem is child:
            return i
    return -1  # Child element not found in the parent

def insert_atomicBT(xmlfile,check,atomicBT):
    tree = etree.parse(xmlfile)
    for child in tree.iter(check):
        check_element = deepcopy(child)
        check_child = child
        check_parent = child.getparent()
    print(check_parent)

    newparent = etree.Element("Fallback", name=check+"_Fallback")
    newparent.append(check_element)
    newparent.append(atomicBT)
    printtree(newparent)
    printtree(check_parent)
    check_parent.replace(check_child,newparent)

    tree.write(xmlfile, pretty_print=True)
    print("XML file updated successfully.")

def append_under_sequence(parent,child):
    sequ = etree.Element("Sequence", name ="Sequence_"+child.tag)
    sequ.append(child)
    parent.append(sequ)
    return parent

def append_under_selector(parent,child):
    sel = etree.Element("Fallback",name=child.tag+"_Fallback")
    sel.append(child)
    parent.append(sel)
    return parent

def update_element_values(element, goal_value, tag_value):
    # Find elements with "goal" attribute and update their value
    for elem in element.xpath('//*[@goal]'):
        elem.set('goal', goal_value)
    
    # Find elements with "tag" attribute and update their value
    for elem in element.xpath('//*[@tag]'):
        elem.set('tag', tag_value)
    
    return element

class TreeParser():
    def __init__(self, xmlfile):
        self.xml = xmlfile

    def update_tree(self):
         self.tree = etree.parse(self.xml)
         self.root = self.tree.getroot()
         self.MainTree = self.root[0]
         return self.root
    
    def write_into_file(self):
        self.tree.write("include/trees/bt.xml", pretty_print=True)
        print("XML file updated successfully.")
    
    def update_child_position(self, child_name, new_parent, new_parent_attrib=None):
        # Create a new parent element
        new_parent_element = etree.Element(new_parent)
        new_parent_element.attrib["name"] = new_parent_attrib

        # Find the parent element
        child = find_child_with_tag(self.MainTree,child_name)
        parent = child.getparent()
        new_parent_element.insert(0,child)
        parent.getparent().replace(parent,new_parent_element)
        self.write_into_file()

    def insert_atomicBT(self,check,atomicBT):
        for child in self.MainTree.iter(check):
            check_element = deepcopy(child)
            check_child = child
            check_parent = child.getparent()

        if check_parent.tag == "Fallback" and check_parent.get("name") == check+"_Fallback":
            # check_parent.set("name",check+"_Fallback")
            check_parent.append(atomicBT)
        else: 
            newparent = etree.Element("Fallback", name=check+"_Fallback")
            newparent.append(check_element)
            newparent.append(atomicBT)
            check_parent.replace(check_child,newparent)
        self.write_into_file()
    
    def append_under_selector(parent,child):
        sel = etree.Element("Fallback",name=child.tag+"_Fallback")
        sel.append(child)
        parent.append(sel)
        return parent

    def add_groot_node(self):
        btwait = '<initialising_tree name="initialising_tree"/>'
        btwait = etree.fromstring(btwait)
        wait_parent = append_under_selector(self.MainTree,btwait)
        self.write_into_file()
        print("Added tree initialise... connect to groot!")

    
    def remove_groot_node(self):
        del self.MainTree[0]
        self.write_into_file()
def main():
    # generate_base_xml()
    # update_child_position("input.xml", "grasped_check", "Fallback", "grasped_check_fallback")

    string = """<BehaviourTree ID="find">
    <Sequence name="find_Sequence">
        <Fallback name="arm_home_Fallback">
            <arm_home_check service_name="PoseCheckService"/>
            <MoveArmAction goal="True"/>
        </Fallback>
        <Decorator ID="RetryUntilSuccessful" num_attempts="4">
            <Action ID="MoveBaseAction" goal="-2.3;-0.5;0.0;0.0;0.0;0.99;0.0" server_name="/move_base" timeout="500"/>
        </Decorator>
    </Sequence>
</BehaviourTree>
"""
    atomic_bt = etree.fromstring(string)
    printtree(atomic_bt)
    # insert_atomicBT("include/trees/bt.xml","grasped_check",atomic_bt)

# main()


