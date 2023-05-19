// header_file.h

#ifndef HEADER_H_
#define HEADER_H_

#include <behaviortree_cpp_v3/action_node.h>
#include <string> 
#include "std_msgs/String.h"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_ros/bt_action_node.h>
#include <behaviortree_ros/bt_service_node.h>

#include <ros/ros.h>
#include "ros/package.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <experimental/filesystem>
#include <move_base_msgs/MoveBaseAction.h>
#include <tree_msgs/BehaviourTreeAction.h>
#include <cstdlib>
#include <ctime>

#include <albert_skills/LookForItemAction.h>
#include <albert_skills/PickAction.h>
#include <albert_skills/PlaceAction.h>

#include <franka_vacuum_gripper/DropOffAction.h>
#include <franka_vacuum_gripper/VacuumAction.h>
#include <franka_vacuum_gripper/StopAction.h>

/// checks services
#include <apriltag_filter/FilterAprilTagDetections.h>
#include <tree_msgs/VacuumCheck.h>
#include <tree_msgs/LocationCheck.h>
#include <tree_msgs/PoseCheck.h>
#include <tree_msgs/HomeCheck.h>  //this one for home pose at the start




using namespace BT;

typedef Tree bt;
BehaviorTreeFactory factory;

#endif // HEADER_FILE_H_
