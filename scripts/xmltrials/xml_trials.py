import xml.etree.ElementTree as ET
import xmltodict

def generate_base_xml():
    # Create the root element
    root = ET.Element("root")
    root.attrib["BTCPP_format"]="4"

    BehaviorTree = ET.SubElement(root,"BehaviorTree")
    BehaviorTree.attrib["ID"]="MainTree"

    # root_Fallback = ET.SubElement(BehaviorTree,"Fallback")
    # root_Fallback.attrib["name"]="Root_Fallback"

    # Create an ElementTree object with the root element
    tree = ET.ElementTree(root)
    # Write the XML tree to a file
    tree.write("input.xml")

def find_in_root(root,wanted_tag,attribute):
    # Access elements and attributes
    # for child in root:
    #     #gets children and their attributes
    #     print(child.tag, child.attrib)
    for elem in root.iter():
        #gets all children and subchildren tag and attributes
        # print(elem.tag,elem.attrib)
        if elem.tag==wanted_tag: 
            # for at in elem.attrib:
            #     if at ==attribute:
            #         print("YOHOHOHOHO")
            return elem

def printxml(root):
    print(ET.tostring(root, encoding='utf8').decode('utf8'))

def find_within_xml(inputxml,attribute,attrib_value,child_tag=None):
    # Parse the XML file
    tree = ET.parse(inputxml)

    # Get the root element
    root = tree.getroot()
    #looks into all children and finds the one you need 
    # you can get the exact attribute you need with the specific attribute you want
    for child in root.iter(child_tag):
        if child.attrib.get(attribute)==attrib_value:
            # print(child.tag,child.attrib)
            return child

def get_child_index(root, child_element):
    for index, element in enumerate(root):
        if element.tag == child_element:
            return index
            # Child element not found
    return -1

def update_child_position(xmlfile, parent_element_name, new_parent, new_parent_name):
    # Parse the XML file
    tree = ET.parse(xmlfile)
    root = tree.getroot()
    parent_element = find_within_xml(xmlfile,"name",parent_element_name)
    print(parent_element)
    # If the parent element is found, update the XML structure
    if parent_element is not None:
        # Create a new parent element
        new_parent_element = ET.Element(new_parent)
        new_parent_element.attrib["name"] = new_parent_name

        # Move the children from the old parent to the new parent
        children = parent_element.getchildren()
        for child in children:
            parent_element.remove(child)
            new_parent_element.append(child)



        # Save the modified XML file
        tree.write(xmlfile)
        print("XML file updated successfully.")
    else:
        print("Parent element not found in the XML file.")

def swap_child_positions(xml_file, child1_goal, child2_goal):
    # Parse the XML file
    tree = ET.parse(xml_file)
    root = tree.getroot()

    # Find the first child element
    child1 = None
    for element in root:
        if element.tag == "child1":
            child1 = element
            break

    # Find the second child element
    child2 = None
    for element in root:
        if element.tag == "child2":
            child2 = element
            break

    # Swap the positions of the children
    index1 = list(root).index(child1)
    index2 = list(root).index(child2)
    print(index1,index2)
    root[index1], root[index2] = root[index2], root[index1]

    # Write the modified XML back to the file
    tree.write(xml_file)

def create_xml_from_string(xml_string):
    # Parse the XML string
    root = ET.fromstring(xml_string)
    # Create an ElementTree object
    tree = ET.ElementTree(root)
    # Write the ElementTree to a file
    return tree
    # tree.write(file_path)

def merge_xml_files(file1, file2, output_file):
    # Parse the first XML file
    tree1 = ET.parse(file1)
    root1 = tree1.getroot()

    # Parse the second XML file
    tree2 = ET.parse(file2)
    root2 = tree2.getroot()

    # Create a new root element

    # # Append elements from the first XML file
    # for element in root1:
    #     merged_root.append(element)

    # # Append elements from the second XML file
    # for element in root2:
    #     merged_root.append(element)
    root1.extend(root2)
    # Create an ElementTree object with the merged root
    merged_tree = ET.ElementTree(root1)
    # Write the merged XML to the output file
    merged_tree.write(output_file)

def get_root_tag(element):
    root = element.getroot()
    return root.tag

def add_under_FB(file1, file2,parent,name, output_file):
    # Parse the first XML file
    tree1 = ET.parse(file1)
    root1 = tree1.getroot()
    # tree2 = ET.parse(file2)
    root2 = file2.getroot()

    for child in root1:
        if child.tag==parent:
            fallback = ET.SubElement(child,"Fallback")
            fallback.attrib["name"]=name+"_fallback"
            fallback.append(root2)

    tree1.write(output_file)

def add_under_seq(file1, file2,parent,name, output_file):
    # Parse the first XML file
    tree1 = ET.parse(file1)
    root1 = tree1.getroot()
    root2 = file2.getroot()

    for child in root1:
        if child.tag==parent:
            sequence = ET.SubElement(child,"Sequence")
            sequence.attrib["name"]=name+"_Sequence"
            sequence.append(root2)

    tree1.write(output_file)

def g_action_seq(file1):
    # Parse the first XML file
    sequence = ET.Element("Sequence")
    for sting in file1:
        xml = create_xml_from_string(sting)
        root = xml.getroot()
        sequence.append(root)

    # printxml(sequence)
    return sequence
    # tree1.write(output_file)  


def add_to_parent(file1, file2,parent,name, output_file):
    # Parse the first XML file
    tree1 = ET.parse(file1)
    root1 = tree1.getroot()
    try:
        file2 = file2.getroot()
    except:
        pass

    for child in root1.findall(".//{}".format(parent)):
        if child.attrib["name"]==name:
            child.append(file2)
    # Create a new root element
    # print(ET.tostring(root1, encoding='utf8').decode('utf8'))
    tree1.write(output_file)

def extract_bt(string):
    '''this function extracts the tree ID and the subtree and returns them
    tree ID can be placed where is needed in the main BT
    subtree can be used to update goal'''
    tree = create_xml_from_string(string)
    root = tree.getroot()
    magi = str("<"+root.tag+" ID='"+root.attrib["ID"]+"'/>")
    return magi,root

def appendit(file,element,output_file):
    tree1 = ET.parse(file)
    root1 = tree1.getroot()
    root2 = element.getroot()
    root1.append(root2)
    tree1.write(output_file)
    

def main():
    # generate_base_xml() #generates the basic of a main BT

#     child = find_within_xml("input.xml","ID","MainTree")
#     print(child)
#     # print(ET.tostring(child, encoding='utf8').decode('utf8'))
     
# ####### Explanation for this script so far ########
# ### you can use find within xml and give the xml file, attribute and attribute value and it will return that child.
# ### Child is returned with all its sub children. 
# ### you can turn string to xml element and retrun that for use using create xml from string
# ### add to parent adds element to an xml by defining the parent's tag (MAYBE NEED TO ADD ATTRIBUTE HERE TOO)
# ### IT MIGHT BE A BETTER IDEA TO MAKE IT A CLASS SINCE WE WILL BE MANIPULATING THE SAME TREE 
# ### AND WE WOULD HAVE ACCESS TO LOWER THE TIMES WE SEARCH PARTY
#     xml_string = '<location_check service_name="location_checkService"></location_check>'
#     file_path = "output.xml"
#     tree = create_xml_from_string(xml_string, file_path)
#     add_to_parent("input.xml",tree,"Fallback",file_path)
#     sub = ET.parse("subtreexml.xml")
#     add_to_parent(file_path,sub,"Fallback",file_path)
    
#     # swap_child_positions("input.xml", "subchild1", "subchild2")
#     # file1 = "input.xml"
#     # filesub = "subtreexml.xml"
#     # output_file = "merged.xml"
#     # file2 = "checkxml.xml"
#     # merge_xml_files(file2,filesub,output_file)

#     # add_under_seq(file1,file2,output_file)
#     # add_to_parent(output_file, filesub,"Sequence", output_file)

    generate_base_xml()


    # sting = '<BehaviorTree ID="MoveBaseTree"> \
    #     <Decorator ID="RetryUntilSuccessful" num_attempts="4"> \
    #         <Action ID="MoveBaseAction" goal="0.0;0.0;0.0;0.0;0.0;0.0;1.0" server_name="/move_base" timeout="500" /> \
    #     </Decorator> \
    # </BehaviorTree> '
    # tree = create_xml_from_string(sting)
    # root = tree.getroot()
    # magi = str("<"+root.tag+" ID='"+root.attrib["ID"]+"'/>")
    # print(magi)
    # mm = create_xml_from_string(magi)
    # print(ET.tostring(root, encoding='utf8').decode('utf8'))



main()










# # def xml_to_dic(input,child,lable):
# #     # Read the XML file
# #     with open(input) as file:
# #         xml_data = file.read()

# #     # Convert XML to dictionary
# #     data_dict = xmltodict.parse(xml_data)
# #     # Modify the desired attribute
# #     data_dict["root"][child]["@"+lable] = "jssj"

# #     # Convert dictionary back to XML
# #     modified_xml = xmltodict.unparse(data_dict)

# #     # Write the modified XML to a file
# #     with open("output.xml", "w") as file:
# #         file.write(modified_xml)
