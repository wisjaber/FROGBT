#include "header.h"
#ifndef BT_CLASSSES_H
#define BT_CLASSSES_H


ros::Publisher monitor_publisher;

void publishMessage(const std::string& message)
{
    // Create a string message
    std_msgs::String msg;
    msg.data = message;

    // Publish the message
    monitor_publisher.publish(msg);
}


class initialising_tree : public BT::SyncActionNode
{
public:
  explicit initialising_tree(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::cout << "Tree is initialising" << std::endl;
    ros::Rate rate(1);
    // Sleep for 3 seconds
    ros::Duration(5.0).sleep();
    return NodeStatus::FAILURE;
  }
};

class MoveBaseAction: public RosActionNode<move_base_msgs::MoveBaseAction>
  {

  public:
  MoveBaseAction(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<move_base_msgs::MoveBaseAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("goal")};
    }

  bool sendGoal(GoalType& goal) override
    {
      // std::string goalto;
      auto goalto = getInput<std::string>("goal");

      std::cout << "setting course towards goal" <<std::endl;
      Pose3D goal_from_port = convertFromString<Pose3D>(goalto.value());
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.pose.position.x=goal_from_port.x;
      goal.target_pose.pose.position.y=goal_from_port.y;
      goal.target_pose.pose.position.z=goal_from_port.z;
      goal.target_pose.pose.orientation.x=goal_from_port.ox;
      goal.target_pose.pose.orientation.y=goal_from_port.oy;
      goal.target_pose.pose.orientation.z=goal_from_port.oz;
      goal.target_pose.pose.orientation.w=goal_from_port.ow;

      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("result received");
        return NodeStatus::SUCCESS;
      }


    virtual NodeStatus onFailedRequest(FailureCause failure) override
    {
      ROS_ERROR("request failed");
      return NodeStatus::FAILURE;
    }

    // void halt() override
    // {
    //   if( status() == NodeStatus::RUNNING )
    //   {
    //     ROS_WARN("halted");
    //     BaseClass::halt();
    //   }
    // }
    
    };

class MoveArmAction: public RosActionNode<tree_msgs::MoveArmAction>
  {

  public:
  MoveArmAction(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<tree_msgs::MoveArmAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("goal")};
    }

  bool sendGoal(GoalType& goal) override
    {
      // std::string goalto;
      auto goalto = getInput<std::string>("goal");

      std::cout << "sending arm home" <<std::endl;

      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("result received");
        return NodeStatus::SUCCESS;
      }


    virtual NodeStatus onFailedRequest(FailureCause failure) override
    {
      ROS_ERROR("request failed");
      return NodeStatus::FAILURE;
    }

    // void halt() override
    // {
    //   if( status() == NodeStatus::RUNNING )
    //   {
    //     ROS_WARN("halted");
    //     BaseClass::halt();
    //   }
    // }
    
    };

class PickAction: public RosActionNode<albert_skills::PickAction>
  {

  public:
  PickAction(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<albert_skills::PickAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("tag")};
    }

  bool sendGoal(GoalType& goal) override
    {
      std::string id_from_port;
      double id;
      getInput<std::string>("tag", id_from_port);
      id = convertFromString<double>(id_from_port);
      std::cout << "picking product with id " << id <<std::endl; 
      goal.goal_id = id;
      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("result received");
        return NodeStatus::SUCCESS;
      }


    virtual NodeStatus onFailedRequest(FailureCause failure) override
    {
      ROS_ERROR("request failed");
      return NodeStatus::FAILURE;
    }

    // void halt() override
    // {
    //   if( status() == NodeStatus::RUNNING )
    //   {
    //     ROS_WARN("halted");
    //     BaseClass::halt();
    //   }
    // }
    
    };

class PlaceAction: public RosActionNode<albert_skills::PlaceAction>
  {

  public:
  PlaceAction(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<albert_skills::PlaceAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("tag")};
    }

  bool sendGoal(GoalType& goal) override
    {
      std::string id_from_port;
      double id;
      getInput<std::string>("tag", id_from_port);
      id = convertFromString<double>(id_from_port);
      std::cout << "PLACING product with id " << id <<std::endl; 
      goal.goal_id = id;
      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("result received");
        return NodeStatus::SUCCESS;
      }


    virtual NodeStatus onFailedRequest(FailureCause failure) override
    {
      ROS_ERROR("request failed");
      return NodeStatus::FAILURE;
    }

    // void halt() override
    // {
    //   if( status() == NodeStatus::RUNNING )
    //   {
    //     ROS_WARN("halted");
    //     BaseClass::halt();
    //   }
    // }
    
    };

class PickFromFloorAction: public RosActionNode<albert_skills::PickAction>
  {

  public:
  PickFromFloorAction(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<albert_skills::PickAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("tag")};
    }

  bool sendGoal(GoalType& goal) override
    {
      std::string id_from_port;
      double id;
      getInput<std::string>("tag", id_from_port);
      id = convertFromString<double>(id_from_port);
      std::cout << "picking product with id " << id <<std::endl; 
      goal.goal_id = id;
      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("result received");
        return NodeStatus::SUCCESS;
      }


    virtual NodeStatus onFailedRequest(FailureCause failure) override
    {
      ROS_ERROR("request failed");
      return NodeStatus::FAILURE;
    }

    // void halt() override
    // {
    //   if( status() == NodeStatus::RUNNING )
    //   {
    //     ROS_WARN("halted");
    //     BaseClass::halt();
    //   }
    // }
    
    };

class ApproachAction: public RosActionNode<tree_msgs::ApproachAction>
  {

  public:
  ApproachAction(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<tree_msgs::ApproachAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("tag"), InputPort<std::string>("offset")};
    }

  bool sendGoal(GoalType& goal) override
    {
      std::string id_from_port, offset_from_port;
      double id, offset;
      getInput<std::string>("tag", id_from_port);
      id = convertFromString<double>(id_from_port);
      getInput<std::string>("offset", offset_from_port);
      offset = convertFromString<double>(offset_from_port);
      std::cout << "approaching product with id " << id <<std::endl; 
      goal.tag_id = id;
      goal.offset = offset;
      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("result received");
        return NodeStatus::SUCCESS;
      }


    virtual NodeStatus onFailedRequest(FailureCause failure) override
    {
      ROS_ERROR("request failed");
      return NodeStatus::FAILURE;
    }

    // void halt() override
    // {
    //   if( status() == NodeStatus::RUNNING )
    //   {
    //     ROS_WARN("halted");
    //     BaseClass::halt();
    //   }
    // }
    
    };

class ScanFloor: public RosActionNode<albert_skills::PickAction>
  {

  public:
  ScanFloor(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<albert_skills::PickAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("tag")};
    }

  bool sendGoal(GoalType& goal) override
    {
      std::string id_from_port;
      double id;
      getInput<std::string>("tag", id_from_port);
      id = convertFromString<double>(id_from_port);
      std::cout << "picking product with id " << id <<std::endl; 
      goal.goal_id = id;
      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("result received");
        return NodeStatus::SUCCESS;
      }


    virtual NodeStatus onFailedRequest(FailureCause failure) override
    {
      ROS_ERROR("request failed");
      return NodeStatus::FAILURE;
    }

    // void halt() override
    // {
    //   if( status() == NodeStatus::RUNNING )
    //   {
    //     ROS_WARN("halted");
    //     BaseClass::halt();
    //   }
    // }
    
    };

// LOOK MORE INTO THE PLACING ACTION, THERE MIGHT BE ERRORS THERE

class VacuumAction: public RosActionNode<franka_vacuum_gripper::VacuumAction>
  {

  public:
  VacuumAction(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<franka_vacuum_gripper::VacuumAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("server_name")};
    }

  bool sendGoal(GoalType& goal) override
    {
      std::cout << "initiating vacuum " <<std::endl; 
      goal.vacuum = 10;
      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("result received");
        return NodeStatus::SUCCESS;
      }


    virtual NodeStatus onFailedRequest(FailureCause failure) override
    {
      ROS_ERROR("request failed");
      return NodeStatus::FAILURE;
    }

    // void halt() override
    // {
    //   if( status() == NodeStatus::RUNNING )
    //   {
    //     ROS_WARN("halted");
    //     BaseClass::halt();
    //   }
    // }
    
    };

class DropOffAction: public RosActionNode<franka_vacuum_gripper::DropOffAction>
  {

  public:
  DropOffAction(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<franka_vacuum_gripper::DropOffAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("server_name")};
    }

  bool sendGoal(GoalType& goal) override
    {
      std::cout << "stopping vacuum " <<std::endl; 
      goal.timeout = 10;
      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("result received");
        return NodeStatus::SUCCESS;
      }


    virtual NodeStatus onFailedRequest(FailureCause failure) override
    {
      ROS_ERROR("request failed");
      return NodeStatus::FAILURE;
    }

    // void halt() override
    // {
    //   if( status() == NodeStatus::RUNNING )
    //   {
    //     ROS_WARN("halted");
    //     BaseClass::halt();
    //   }
    // }
    
    };


class RecoveryAction: public RosActionNode<tree_msgs::RecoveryAction>
  {

  public:
  RecoveryAction(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<tree_msgs::RecoveryAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("action"),InputPort<std::string>("idx")};
    }

  bool sendGoal(GoalType& goal) override
    {
      std::string action_name;
      std::string index;
      getInput<std::string>("action", action_name);
      getInput<std::string>("idx", index);
      goal.action = action_name;
      goal.index = index;
      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("result received");
        return NodeStatus::SUCCESS;
      }


    virtual NodeStatus onFailedRequest(FailureCause failure) override
    {
      ROS_ERROR("request failed");
      return NodeStatus::FAILURE;
    }

    // void halt() override
    // {
    //   if( status() == NodeStatus::RUNNING )
    //   {
    //     ROS_WARN("halted");
    //     BaseClass::halt();
    //   }
    // }
    
    };


class tag_detection_check: public RosServiceNode<apriltag_filter::FilterAprilTagDetections>
  {
  public:
    int id;
    tag_detection_check( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<apriltag_filter::FilterAprilTagDetections>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<int>("tag"),InputPort<std::string>("index")};
    }

    void sendRequest(RequestType& request) override
    {
      getInput("tag", request.tag_id);
      id = request.tag_id;
      ROS_INFO("sending detection request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if( rep.filtered_pose.position.x != 0)
      {
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_ERROR("unable to detect product with id: %d", id);
        auto idx = getInput<std::string>("index");
        publishMessage("tag_detection_check,"+idx.value());
        return NodeStatus::FAILURE;
      }

        return NodeStatus::SUCCESS;

    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

class vacuum_check: public RosServiceNode<tree_msgs::VacuumCheck>
  {
  public:
    vacuum_check( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tree_msgs::VacuumCheck>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("index")};
    }

    void sendRequest(RequestType& request) override
    {
      request.VacuumCheck = true;
      ROS_INFO("sending vacuum check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(rep.response)
      {
        ROS_ERROR("product in gripper");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_ERROR("product not in gripper");
        auto idx = getInput<std::string>("index");
        publishMessage("vacuum_check,"+idx.value());
        return NodeStatus::FAILURE;
      }

        return NodeStatus::SUCCESS;

    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

class drop_check: public RosServiceNode<tree_msgs::VacuumCheck>
  {
  public:
    drop_check( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tree_msgs::VacuumCheck>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("index")};
    }

    void sendRequest(RequestType& request) override
    {
      request.VacuumCheck = true;
      ROS_INFO("sending vacuum check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(not rep.response)
      {
        ROS_ERROR("gripper is full");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_ERROR("gripper is empty");
        auto idx = getInput<std::string>("index");
        publishMessage("drop_check,"+idx.value());
        return NodeStatus::FAILURE;
      }

        return NodeStatus::SUCCESS;

    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

class location_check: public RosServiceNode<tree_msgs::LocationCheck>
  {
  public:
    location_check( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tree_msgs::LocationCheck>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      // return  {InputPort<std::string>("service_name")};
      return  {InputPort<std::string>("goal"), InputPort<std::string>("index")};
    }

    void sendRequest(RequestType& request) override
    {
      auto goalto = getInput<std::string>("goal");
      Pose3D goal_from_port = convertFromString<Pose3D>(goalto.value());
      request.LocationCheck.position.x = goal_from_port.x;
      request.LocationCheck.position.y = goal_from_port.y;
      request.LocationCheck.position.z = goal_from_port.z;
      request.LocationCheck.orientation.x = goal_from_port.ox;
      request.LocationCheck.orientation.y = goal_from_port.oy;
      request.LocationCheck.orientation.z = goal_from_port.oz;
      request.LocationCheck.orientation.w = goal_from_port.ow;
      ROS_INFO("sending location check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(rep.response)
      {
        ROS_INFO("albert is at desired location");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_INFO("albert is not at desired location");
        auto idx = getInput<std::string>("index");
        // checkfail.data = "location_check";
        publishMessage("location_check,"+idx.value());
        return NodeStatus::FAILURE;
      }

        return NodeStatus::SUCCESS;

    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

class deliver_check: public RosServiceNode<tree_msgs::DeliverCheck>
  {
  public:
    deliver_check( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tree_msgs::DeliverCheck>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      // return  {InputPort<std::string>("service_name")};
      return  {InputPort<std::string>("goal"), InputPort<std::string>("index")};
    }

    void sendRequest(RequestType& request) override
    {
      auto goalto = getInput<std::string>("goal");
      Pose3D goal_from_port = convertFromString<Pose3D>(goalto.value());
      request.LocationCheck.position.x = goal_from_port.x;
      request.LocationCheck.position.y = goal_from_port.y;
      request.LocationCheck.position.z = goal_from_port.z;
      request.LocationCheck.orientation.x = goal_from_port.ox;
      request.LocationCheck.orientation.y = goal_from_port.oy;
      request.LocationCheck.orientation.z = goal_from_port.oz;
      request.LocationCheck.orientation.w = goal_from_port.ow;
      ROS_INFO("sending delivered check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(rep.response)
      {
        ROS_INFO("albert has delivered product at desired location");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_INFO("albert has not yet delivered product to desired location");
        // checkfail.data = "location_check";
        auto idx = getInput<std::string>("index");
        publishMessage("deliver_check,"+idx.value());
        return NodeStatus::FAILURE;
      }

        return NodeStatus::SUCCESS;

    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

class arm_pose_check: public RosServiceNode<tree_msgs::PoseCheck>
  {
  public:
    arm_pose_check( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tree_msgs::PoseCheck>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("index")};
    }

    void sendRequest(RequestType& request) override
    {
      // request.PoseCheck = PoseStamped(); // insert location pose as a request
        // request = PoseCheckRequest()
        // response = PoseCheckService(request)
      ROS_INFO("sending location check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(rep.response)
      {
        ROS_ERROR("albert arm is in the desired pose");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_ERROR("albert is not in the desired pose");
        return NodeStatus::FAILURE;
      }

        return NodeStatus::SUCCESS;

    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

class arm_home_check: public RosServiceNode<tree_msgs::PoseCheck>
  {
  public:
    arm_home_check( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tree_msgs::PoseCheck>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("service_name")};
    }

    void sendRequest(RequestType& request) override
    {
      // request.PoseCheck = PoseStamped(); // insert location pose as a request
        request.PoseCheck.header.frame_id="map";
        request.PoseCheck.pose.position.x = -2.5;
        request.PoseCheck.pose.position.y=-0.5;
        request.PoseCheck.pose.position.z=1.39;
        request.PoseCheck.pose.orientation.x=0.69;
        request.PoseCheck.pose.orientation.y=-0.0;
        request.PoseCheck.pose.orientation.z=-0.7;
        request.PoseCheck.pose.orientation.w=0.0;
      ROS_INFO("sending location check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(rep.response)
      {
        ROS_ERROR("albert arm is in the desired pose");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_ERROR("albert is not in the desired pose");
        return NodeStatus::FAILURE;
      }

        return NodeStatus::SUCCESS;

    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

class picked_check: public RosServiceNode<tree_msgs::product_stateCheck>
  {
  public:
    picked_check( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tree_msgs::product_stateCheck>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<int>("tag"),InputPort<std::string>("index")};
    }

    void sendRequest(RequestType& request) override
    {
      getInput("tag", request.tag);
      request.state= "picked";
      ROS_INFO("sending picked check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(rep.response)
      {
        ROS_ERROR("product in gripper");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_ERROR("product not in gripper");
        auto idx = getInput<std::string>("index");
        publishMessage("picked_check,"+idx.value());
        return NodeStatus::FAILURE;
      }

        return NodeStatus::SUCCESS;

    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

class placed_check: public RosServiceNode<tree_msgs::product_stateCheck>
  {
  public:
    placed_check( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tree_msgs::product_stateCheck>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<int>("tag"),InputPort<std::string>("index")};
    }

    void sendRequest(RequestType& request) override
    {
      getInput("tag", request.tag);
      request.state= "placed";
      ROS_INFO("sending placed check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(rep.response)
      {
        ROS_ERROR("product in gripper");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_ERROR("product not in gripper");
        auto idx = getInput<std::string>("index");
        publishMessage("placed_check,"+idx.value());
        return NodeStatus::FAILURE;
      }

        return NodeStatus::SUCCESS;

    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

class closeCheck: public RosServiceNode<tree_msgs::product_stateCheck>
  {
  public:
    closeCheck( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tree_msgs::product_stateCheck>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("index")};
    }

    void sendRequest(RequestType& request) override
    {
      // doesn't matter cos we want it to always fail (no check for this)
      request.tag = 20;
      request.state= "flying";
      ROS_INFO("sending close check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(rep.response)
      {
        ROS_ERROR("HOWWWWWWWW????");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_ERROR("Agent is not close");
        auto idx = getInput<std::string>("index");
        publishMessage("closeCheck,"+idx.value());
        return NodeStatus::FAILURE;
      }

        return NodeStatus::SUCCESS;

    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

#endif