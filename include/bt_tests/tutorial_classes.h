#include "tutorialheader.h"
#ifndef TUTORIAL_CLASSSES_H
#define TUTORIAL_CLASSSES_H


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
    ros::Duration(3.0).sleep();
    return NodeStatus::FAILURE;
  }
};

// ACTIONS SKILLS
class DriveAction: public RosActionNode<tree_msgs::MockAction>
  {

  public:
  DriveAction(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<tree_msgs::MockAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("goal")};
    }

  bool sendGoal(GoalType& goal) override
    {
      // std::string goalto;
      auto goalto = getInput<std::string>("goal");

      std::cout << "setting course towards goal" <<std::endl;
      goal.location = goalto.value();
      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("Agent is at desired location");
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

class PickAction: public RosActionNode<tree_msgs::MockAction>
  {

  public:
  PickAction(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<tree_msgs::MockAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  { };
    }

  bool sendGoal(GoalType& goal) override
    {
      // std::string goalto;
    //   auto goalto = getInput<std::string>("goal");

      std::cout << "attempting to pick" <<std::endl;
      goal.location = "picked";
      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("Object is picked");
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

class PlaceAction: public RosActionNode<tree_msgs::MockAction>
  {

  public:
  PlaceAction(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<tree_msgs::MockAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  { };
    }

  bool sendGoal(GoalType& goal) override
    {
      // std::string goalto;
    //   auto goalto = getInput<std::string>("goal");

      std::cout << "attempting to place" <<std::endl;
      goal.location = "placed";
      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("Object is placed");
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

class DetectAction: public RosActionNode<tree_msgs::MockAction>
  {

  public:
  DetectAction(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
  RosActionNode<tree_msgs::MockAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
      return  {};
    }

  bool sendGoal(GoalType& goal) override
    {
      // std::string goalto;
    //   auto goalto = getInput<std::string>("goal");

      std::cout << "detecting..." <<std::endl;
      goal.location = "detected";
      ROS_INFO("sending request");
      return true;
    }

  NodeStatus onResult( const ResultType& res) override
    {
      ROS_INFO("Object is detected");
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

// SERVICES CHECKS
class locationCheck: public RosServiceNode<tree_msgs::stateCheck>
  {
  public:
    locationCheck( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tree_msgs::stateCheck>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      // return  {InputPort<std::string>("service_name")};
      return  {InputPort<std::string>("goal"),
      InputPort<std::string>("index")};
    }

    void sendRequest(RequestType& request) override
    {
        auto goalto = getInput<std::string>("goal");
        request.state = goalto.value();
        ROS_INFO("sending location check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(rep.response)
      {
        ROS_INFO("agent is at desired location");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_INFO("agent is not at desired location");
        auto idx = getInput<std::string>("index");
        // checkfail.data = "location_check";
        publishMessage("locationCheck,"+idx.value());
        return NodeStatus::FAILURE;
      }
    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

class pickCheck: public RosServiceNode<tree_msgs::stateCheck>
  {
  public:
    pickCheck( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tree_msgs::stateCheck>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("index")};
    //   return  {InputPort<std::string>("goal")};
    }

    void sendRequest(RequestType& request) override
    {
        // auto goalto = getInput<std::string>("goal");
        request.state = "picked";
        ROS_INFO("sending pick check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(rep.response)
      {
        ROS_INFO("object is picked");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_INFO("object is not picked");
        auto idx = getInput<std::string>("index");
        // checkfail.data = "location_check";
        publishMessage("pickCheck,"+idx.value());
        return NodeStatus::FAILURE;
      }
    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

class placeCheck: public RosServiceNode<tree_msgs::stateCheck>
  {
  public:
    placeCheck( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tree_msgs::stateCheck>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("index")};
    //   return  {InputPort<std::string>("goal")};
    }

    void sendRequest(RequestType& request) override
    {
        request.state = "placed";
        ROS_INFO("sending place check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(rep.response)
      {
        ROS_INFO("object is placed");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_INFO("object is not placed");
        // checkfail.data = "location_check";
        auto idx = getInput<std::string>("index");
        publishMessage("placeCheck,"+idx.value());
        return NodeStatus::FAILURE;
      }
    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

class detectCheck: public RosServiceNode<tree_msgs::stateCheck>
  {
  public:
    detectCheck( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<tree_msgs::stateCheck>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<std::string>("index")};
    //   return  {InputPort<std::string>("goal")};
    }

    void sendRequest(RequestType& request) override
    {
        request.state = "detected";
        ROS_INFO("sending detect check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(rep.response)
      {
        ROS_INFO("object is detected");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_INFO("object is not detected");
        auto idx = getInput<std::string>("index");
        // checkfail.data = "location_check";
        publishMessage("detectCheck,"+idx.value());
        return NodeStatus::FAILURE;
      }
    }

    virtual NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
      ROS_ERROR("request failed %d", static_cast<int>(failure));
      return NodeStatus::FAILURE;
    }


  };

#endif
