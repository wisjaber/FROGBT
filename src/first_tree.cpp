#include "../include/bt_tests/header.h"
#include "../include/bt_tests/bt_functions.h"
#include "../include/bt_tests/bt_classes.h"
#include "../include/bt_tests/bt_generation_action.h"
#include "../include/bt_tests/bt_actionserver.h"




int main(int argc, char **argv)
{
  ros::init(argc, argv, "recovery_bt");
  ros::NodeHandle nh;
  monitor_publisher = nh.advertise<std_msgs::String>("/tree_monitor", 1, true);


    factory.registerNodeType<initialising_tree>("initialising_tree");
    RegisterRosAction<MoveBaseAction>(factory, "MoveBaseAction", nh);
    RegisterRosAction<ApproachAction>(factory, "ApproachAction", nh);
    RegisterRosAction<ScanFloor>(factory, "ScanFloor", nh);
    RegisterRosAction<MoveArmAction>(factory, "MoveArmAction", nh);
    RegisterRosAction<PickAction>(factory, "PickAction", nh);
    RegisterRosAction<PickFromFloorAction>(factory, "PickFromFloorAction", nh);
    RegisterRosAction<PlaceAction>(factory, "PlaceAction", nh);

    RegisterRosAction<RecoveryAction>(factory, "RecoveryAction", nh);

    RegisterRosService<picked_check>(factory, "picked_check", nh);
    RegisterRosService<placed_check>(factory, "placed_check", nh);

    RegisterRosService<tag_detection_check>(factory, "tag_detection_check", nh);
    RegisterRosService<location_check>(factory, "location_check", nh);
    RegisterRosService<vacuum_check>(factory, "vacuum_check", nh);
    RegisterRosService<drop_check>(factory, "drop_check", nh);
    RegisterRosService<arm_home_check>(factory, "arm_home_check", nh);
    RegisterRosService<deliver_check>(factory, "deliver_check", nh);


    // Setup your tree correctly to use relative paths
    // add the xml file with your tree in config folder
    // in getPath input put your package name
    // in relative just edit the name of the xml file to your desired tree
    // if you are following ros folder structure it should work correctly

    // register_subtrees("bt_tests");


  BtGeneration tree_generation_server("bt_recovery_server");
  std::cout<<"BT recovery server started"<<std::endl;
  //Tree won't start now until it gets a command from the action client
  ros::spin();
  return 0;

}