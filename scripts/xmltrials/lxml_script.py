from lxml import etree
from copy import deepcopy

def generate_base_xml(name):
    # Create the root element
    root = etree.Element("root", main_tree_to_execute="BehaviorTree")
    BehaviorTree = etree.SubElement(root,"BehaviorTree", ID="BehaviorTree")
    tree = etree.ElementTree(root)
    tree.write(f"include/trees/{name}.xml", pretty_print=True)

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

def append_sequence_goals(child):
    sequ = etree.Element("Sequence", name ="Goal_Checks_Sequence")
    for el in child:
        sequ.append(el)
    # parent.append(sequ)
    return sequ

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


def update_element_goals(element, goal_value):
    # Find elements with "goal" attribute and update their value
    for elem in element.xpath('//*[@goal]'):
        elem.set('goal', goal_value)   
    return element

def get_goals_from_check(element):
        for elem in element.xpath('//*[@goal]'):
            goal_value = elem.get('goal')
            return goal_value

def testymctestface(check,idx,root):
        # root = etree.fromstring(stringxml)
        print("entering the search loop")
        for child in root.iter(check):
            if "index" in child.attrib:
                print("found something with index")
                if int(child.attrib["index"]) == int(idx):
                    print("INDEX BEFORE INSERTING BT",idx)
                    check_element = deepcopy(child)
                    check_child = child
                    check_parent = child.getparent()
                    print("check element",printtree(check_element))
                    print(check_child.tag,check_child.get("name"),check_parent.tag,check_parent.get("name"))
                    return check_element, check_child,check_parent
            else:
                print("CHECK HAS NO INDEX",list(child))

def failed_action(action,idx,root):
        # root = etree.fromstring(stringxml)
        print("entering the search loop")
        for child in root.iter("Fallback"):
            print(child.attrib["name"])
            if child.attrib["name"] == action+"_Fallback_branch":
                print(child.attrib["name"])
                action_element = deepcopy(child)
                action_child = child
                action_parent = child.getparent()
                print("action element",printtree(action_element))
                print(action_child.tag,action_child.get("name"),action_parent.tag,action_parent.get("name"))
                return action_element, action_child,action_parent
           

class TreeParser():
    def __init__(self, xmlfile):
        self.xml = xmlfile

    def update_tree(self):
         self.tree = etree.parse(self.xml)
         self.root = self.tree.getroot()
         self.MainTree = self.root[0]
         return self.root
    
    def write_into_file(self):
        self.tree.write(self.xml, pretty_print=True)
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

    def insert_atomicBT(self,check,idx,atomicBT):
        # repeated_checks = []
        check_element, check_child,check_parent = testymctestface(check,idx,self.root)
        if check_parent.tag == "Fallback" and check_parent.get("name") == check+"_Fallback":
            goal_value = get_goals_from_check(check_element)
            if goal_value is not None:
                atomicBT = update_element_goals(atomicBT,goal_value)
            check_parent.append(atomicBT)
        else: 
            newparent = etree.Element("Fallback", name=check+"_Fallback")
            newparent.append(check_element)
            goal_value = get_goals_from_check(check_element)
            if goal_value is not None:
                atomicBT = update_element_goals(atomicBT,goal_value)
            newparent.append(atomicBT)
            check_parent.replace(check_child,newparent)
        # printtree(check)
        self.write_into_file()
        # else:
        #     print("KILL IT WITH FIRE")
              
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
    # testymctestface("locationCheck",3)

#     stringxml = '''<root main_tree_to_execute="BehaviorTree">
#   <BehaviorTree ID="BehaviorTree"><Fallback name="placeCheck_Fallback"><placeCheck service_name="stateCheck" index="1"/><Sequence name="Place_sequence"><Fallback name="pickCheck_Fallback"><pickCheck service_name="stateCheck" index="2"/><Sequence name="Pick_sequence"><Fallback name="detectCheck_Fallback"><detectCheck service_name="stateCheck" index="4"/><Sequence name="Detect_sequence"><Fallback name="locationCheck_Fallback"><locationCheck service_name="locationCheck" goal="PickingLocation" index="5"/><Sequence name="Drive_sequence"><Action ID="DriveAction" server_name="/Drive_fake" goal="PickingLocation" timeout="500"/></Sequence></Fallback><Action ID="DetectAction" server_name="/Detect_fake" timeout="500"/></Sequence></Fallback><Action ID="PickAction" server_name="/Pick_fake" timeout="500"/></Sequence></Fallback><locationCheck service_name="locationCheck" goal="PlacingLocation" index="3"/><Action ID="PlaceAction" server_name="/Place_fake" timeout="500"/></Sequence></Fallback></BehaviorTree>
# </root>'''
#     xmlfile = etree.fromstring(stringxml)
    BTparsed = TreeParser("include/trees/bt.xml")
    tree = BTparsed.update_tree()
    string = """<Sequence name="find_Sequence">
        <Fallback name="arm_home_Fallback">
            <arm_home_check service_name="PoseCheckService"/>
            <MoveArmAction goal="True"/>
        </Fallback>
            <Action ID="MoveBaseAction" goal="-2.3;-0.5;0.0;0.0;0.0;0.99;0.0" server_name="/move_base" timeout="500"/>
    </Sequence>
"""
    atomic_bt = etree.fromstring(string)
    BTparsed.insert_atomicBT("locationCheck",5,atomic_bt)
    # insert_atomicBT("include/trees/bt.xml","locationCheck",3,atomic_bt)
#     printtree(atomic_bt)
    # insert_atomicBT("include/trees/bt.xml","grasped_check",atomic_bt)

# main()


