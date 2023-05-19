#include "header.h"
#ifndef BT_CLASSSES_H
#define BT_CLASSSES_H


NodeStatus initialising_tree()
  {
    std::cout << "Tree is initialising" << std::endl;
    return NodeStatus::SUCCESS;
  }

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
      std::string goalto;

      getInput<std::string>("goal", goalto);

      std::cout << "setting course towards" << goalto<<std::endl;
      // goal.action = go_to_object_action;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.pose.position.x=0;
      goal.target_pose.pose.position.y=0;
      goal.target_pose.pose.position.z=0;
      goal.target_pose.pose.orientation.x=0;
      goal.target_pose.pose.orientation.y=0;
      goal.target_pose.pose.orientation.z=0;
      goal.target_pose.pose.orientation.w=1;

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
      return  {InputPort<std::string>("id")};
    }

  bool sendGoal(GoalType& goal) override
    {
      std::string id_from_port;
      double id;
      getInput<std::string>("id", id_from_port);
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
      return  {InputPort<std::string>("id")};
    }

  bool sendGoal(GoalType& goal) override
    {
      std::string id_from_port;
      double id;
      getInput<std::string>("id", id_from_port);
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

class tag_detection_check: public RosServiceNode<apriltag_filter::FilterAprilTagDetections>
  {
  public:
    int id;
    tag_detection_check( ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration & conf):
    RosServiceNode<apriltag_filter::FilterAprilTagDetections>(handle, node_name, conf) {}

    static PortsList providedPorts()
    {
      return  {InputPort<int>("tag")};
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
      return  {InputPort<std::string>("service_name")};
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
      return  {InputPort<std::string>("service_name")};
    }

    void sendRequest(RequestType& request) override
    {
      // request.LocationCheck = Pose(); // insert location pose as a request
      ROS_INFO("sending location check request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
      ROS_INFO("response received");
      if(rep.response)
      {
        ROS_ERROR("albert is at desired location");
        return NodeStatus::SUCCESS;
      }
      else{
        ROS_ERROR("albert is not at desired location");
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
      return  {InputPort<std::string>("service_name")};
    }

    void sendRequest(RequestType& request) override
    {
      // request.PoseCheck = PoseStamped(); // insert location pose as a request
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

#endif
