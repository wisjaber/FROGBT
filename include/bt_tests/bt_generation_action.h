#include "header.h"
// #include "behaviortree_cpp/loggers/bt_observer.h"

using namespace BT;
typedef BT::Tree bt;

class BtGeneration
{
    protected:
    ros::NodeHandle n_;
    actionlib::SimpleActionServer<tree_msgs::GeneratebtAction> as_;  
    tree_msgs::GeneratebtResult result_;
    std::string action_name_;
    bool preempted_;
    tree_msgs::GeneratebtGoal current_goal_;
    bool isNewGoalAvailable_;
    public:
    bt tree_;
    NodeStatus status = BT::NodeStatus::IDLE;
    BtGeneration(std::string name):
    as_(n_, name, boost::bind(&BtGeneration::executeCB, this ,_1), false),
    action_name_(name), preempted_(false)
    {
        as_.start();
    }

    void goalCB()
      {
        // Accept the new goal
        current_goal_ = *(as_.acceptNewGoal());

        // Set the flag to indicate that a new goal is available
        isNewGoalAvailable_ = true;
      }


  void executeCB(const tree_msgs::GeneratebtGoalConstPtr& goal)  
  {
    std::cout<< "command recieved"<<std::endl;
    bool success=true;
    ros::Rate loop_rate(10);

    // Reset the flag indicating that a new goal is available
    isNewGoalAvailable_ = false;

    // Set the flag indicating that the goal is active
    bool isGoalActive = true;

    tree_ = Assigntree(goal->xml);
    printTreeRecursively(tree_.rootNode());

// // PRINT TREE ///////////////////////////////////////////////
    // TreeObserver observer(tree_);

//     std::map<uint16_t, std::string> ordered_UID_to_path;
//     for(const auto& [name, uid]: observer.pathToUID()) {
//       ordered_UID_to_path[uid] = name;
//     }

//     for(const auto& [uid, name]: ordered_UID_to_path) {
//       std::cout << uid << " -> " << name << std::endl;
//     }
//////////////////////////////////////////////////

    PublisherZMQ publisher_zmq(tree_);
    status = NodeStatus::IDLE;
    std::cout << "Tree Started" << std::endl;

    // Loop until the goal is complete or preempted
    while (ros::ok() && isGoalActive && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
    {
      // Check for a new goal
      if (isNewGoalAvailable_)
      {
        // Check if the new goal is the same as the current goal
        if (current_goal_.xml == goal->xml)
        {
          // Reset the flag indicating that a new goal is available
          isNewGoalAvailable_ = false;
        }
        else
        {
          // Indicate that the goal was preempted
          as_.setPreempted();
          // Reset the flag indicating that a new goal is available
          isNewGoalAvailable_ = false;
        }
      }
          
      status = tree_.tickRoot();
/////////////////////////////////////////////////////////////////////
       // You can access a specific statistic, using is full path or the UID
      // const auto& last_action_stats = observer.getStatistics("last_action");
      // assert(last_action_stats.transitions_count > 0);

      // std::cout << "----------------" << std::endl;
      // // print all the statistics
      // for(const auto& [uid, name]: ordered_UID_to_path) {
      //   const auto& stats = observer.getStatistics(uid);

      //   std::cout << "[" << name
      //             << "] \tT/S/F:  " << stats.transitions_count
      //             << "/" << stats.success_count
      //             << "/" << stats.failure_count
      //             << std::endl;
      // }

////////////////////////////////////////////////////////////////////
      // Check if the goal has been preempted
      if (as_.isPreemptRequested())
      {
        result_.success = false;
        // Indicate that the goal was preempted
        as_.setPreempted();
        // Set the flag indicating that the goal is no longer active
        isGoalActive = false;
        tree_.haltTree();
        success=false;
        break; 
      }
    }
      std::cout<<status<<std::endl;

        if (status==NodeStatus::FAILURE)
        {
          success = false;
          result_.success=false;
        }
        if (status==NodeStatus::SUCCESS)
        {
          success = true;
          result_.success=true;
        }

    if(success)
    {
      result_.success = true;
      std::cout<<"mission passed"<<std::endl;
      std::cout<<result_<<std::endl;
      as_.setSucceeded(result_);
    }
    if(!success)
    {
      result_.success = false;
      std::cout<<"mission failed"<<std::endl;
      as_.setAborted(result_);
    }
  }

};


// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "bt_generation_server");

//   ros::NodeHandle nh;
//   BtGeneration server(nh, "bt_generation_server");
//   server.start();

//   ros::spin();

//   return 0;
// }