#include "../include/bt_tests/header.h"
#include "../include/bt_tests/bt_functions.h"
#include "../include/bt_tests/bt_classes.h"
#include "../include/bt_tests/bt_actionserver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_bt");
  ros::NodeHandle nh;

    factory.registerSimpleCondition("initialising_tree", std::bind(initialising_tree));
    RegisterRosAction<MoveBaseAction>(factory, "MoveBaseAction", nh);
    RegisterRosAction<PickAction>(factory, "PickAction", nh);
    RegisterRosAction<PlaceAction>(factory, "PlaceAction", nh);
    RegisterRosService<tag_detection_check>(factory, "tag_detection_check", nh);
    RegisterRosService<vacuum_check>(factory, "vacuum_check", nh);



    // Setup your tree correctly to use relative paths
    // add the xml file with your tree in config folder
    // in getPath input put your package name
    // in relative just edit the name of the xml file to your desired tree
    // if you are following ros folder structure it should work correctly

    register_subtrees("bt_tests");
  //   auto tree = factory.createTree("MainTree");
  //   PublisherZMQ publisher_zmq(tree);

  //   NodeStatus status = NodeStatus::IDLE;

  //   while (ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
  //   {
  //       status = tree.tickRoot();
  //   }
  // return 0;

  BtServer tree_server("btserver");
  tree_server.tree_=factory.createTree("MainTree");
  
  // adding the following line opens a port to publish the tree to be monitored using groot
  PublisherZMQ publisher_zmq(tree_server.tree_);

  //Tree won't start now until it gets a command from the action client
  ros::spin();
  return 0;

}