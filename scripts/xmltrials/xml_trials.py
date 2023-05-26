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

def find_within_xml(inputxml,attribute,attrib_value,child_tag=None):
    # Parse the XML file
    tree = ET.parse(inputxml)

    # Get the root element
    root = tree.getroot()

    # Access elements and attributes
    for child in root:
        #gets children and their attributes
        print(child.tag, child.attrib)
    for elem in root.iter():
        #gets all children and subchildren tag and attributes
        print(elem.tag,elem.attrib)

    #prints the whole xml as a tree string
    print(ET.tostring(root, encoding='utf8').decode('utf8'))

    #looks into all children and finds the one you need 
    # you can get the exact attribute you need with the specific attribute you want
    for child in root.iter(child_tag):
        if child.attrib.get(attribute)==attrib_value:
            print(child.tag,child.attrib)
            return child

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

def create_xml_from_string(xml_string, file_path):
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

def add_under_FB(file1, file2, output_file):
    # Parse the first XML file
    tree1 = ET.parse(file1)
    root1 = tree1.getroot()
    tree2 = ET.parse(file2)
    root2 = tree2.getroot()

    for child in root1:
        if child.tag=="BehaviorTree" and child.attrib["ID"]=="MainTree":
            fallback = ET.SubElement(child,"Fallback")
            fallback.attrib["name"]="root1_fallback"
            fallback.append(root2)

    tree1.write(output_file)

def add_under_seq(file1, file2, output_file):
    # Parse the first XML file
    tree1 = ET.parse(file1)
    root1 = tree1.getroot()
    tree2 = ET.parse(file2)
    root2 = tree2.getroot()

    for child in root1:
        if child.tag=="BehaviorTree" and child.attrib["ID"]=="MainTree":
            sequence = ET.SubElement(child,"Sequence")
            sequence.attrib["name"]="root1_Sequence"
            sequence.append(root2)

    tree1.write(output_file)

def add_to_parent(file1, file2,parent, output_file):
    # Parse the first XML file
    tree1 = ET.parse(file1)
    root1 = tree1.getroot()

    root2 = file2.getroot()

    for child in root1.findall(".//{}".format(parent)):
        child.append(root2)
    # Create a new root element

    tree1.write(output_file)

def main():
    # generate_base_xml() #generates the basic of a main BT

    child = find_within_xml("input.xml","ID","MainTree")
    print(child)
    # print(ET.tostring(child, encoding='utf8').decode('utf8'))
     
####### Explanation for this script so far ########
### you can use find within xml and give the xml file, attribute and attribute value and it will return that child.
### Child is returned with all its sub children. 
### you can turn string to xml element and retrun that for use using create xml from string
### add to parent adds element to an xml by defining the parent's tag (MAYBE NEED TO ADD ATTRIBUTE HERE TOO)
### IT MIGHT BE A BETTER IDEA TO MAKE IT A CLASS SINCE WE WILL BE MANIPULATING THE SAME TREE 
### AND WE WOULD HAVE ACCESS TO LOWER THE TIMES WE SEARCH PARTY
    xml_string = '<location_check service_name="location_checkService"></location_check>'
    file_path = "output.xml"
    tree = create_xml_from_string(xml_string, file_path)
    add_to_parent("input.xml",tree,"Fallback",file_path)
    sub = ET.parse("subtreexml.xml")
    add_to_parent(file_path,sub,"Fallback",file_path)
    
    # swap_child_positions("input.xml", "subchild1", "subchild2")
    # file1 = "input.xml"
    # filesub = "subtreexml.xml"
    # output_file = "merged.xml"
    # file2 = "checkxml.xml"
    # merge_xml_files(file2,filesub,output_file)

    # add_under_seq(file1,file2,output_file)
    # add_to_parent(output_file, filesub,"Sequence", output_file)
main()



# def xml_to_dic(input,child,lable):
#     # Read the XML file
#     with open(input) as file:
#         xml_data = file.read()

#     # Convert XML to dictionary
#     data_dict = xmltodict.parse(xml_data)
#     # Modify the desired attribute
#     data_dict["root"][child]["@"+lable] = "jssj"

#     # Convert dictionary back to XML
#     modified_xml = xmltodict.unparse(data_dict)

#     # Write the modified XML to a file
#     with open("output.xml", "w") as file:
#         file.write(modified_xml)
