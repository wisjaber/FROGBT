#include "header.h"
// #include "tutorialheader.h"


#ifndef BT_FUNCTIONS_H
#define BT_FUNCTIONS_H

// class and function definitions go here
namespace fs = std::experimental::filesystem;

bt Assigntree(const std::string &xml)
{
  /**
   * This function takes xml file name and BT factory and creates the tree.
  */
    bt tree= factory.createTreeFromFile(xml);
    return tree;
}

std::string setpath(std::string tree)
{
  /**
* This function sets the path to the tree xml from the relative path of the package
*/
  std::string path = ros::package::getPath("bt_tests");
  std::string relative = "/config/";
  relative.append(tree);
  path.append(relative);
  return path;
}

void register_subtrees(std::string pkgname)
{
    std::string path = ros::package::getPath(pkgname);
    std::string relative = "/include/trees/";
    path.append(relative);

    // using std::experimental::filesystem::directory_iterator;
    for (auto const& entry : fs::directory_iterator(path)) 
    {
        if( entry.path().extension() == ".xml")
        {
        factory.registerBehaviorTreeFromFile(entry.path().string());
        }
    }
}

#endif