#include "header.h"
// #include "tutorialheader.h"


#ifndef BT_ACTIONSERVER_H
#define BT_ACTIONSERVER_H

using namespace BT;
typedef BT::Tree bt;

class BtServer
{
    protected:
    ros::NodeHandle n_;
    actionlib::SimpleActionServer<tree_msgs::BehaviourTreeAction> as_;  
    tree_msgs::BehaviourTreeResult result_;
    std::string action_name_;
    bool preempted_;
    tree_msgs::BehaviourTreeGoal current_goal_;
    bool isNewGoalAvailable_;
    public:
    bt tree_;
    NodeStatus status = BT::NodeStatus::IDLE;
    BtServer(std::string name):
    as_(n_, name, boost::bind(&BtServer::executeCB, this ,_1), false),
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


void executeCB(const tree_msgs::BehaviourTreeGoalConstPtr& goal)  
{
  std::cout<< "command recieved"<<std::endl;
  bool success=true;
  ros::Rate loop_rate(10);
  // Reset the flag indicating that a new goal is available
  isNewGoalAvailable_ = false;

  // Set the flag indicating that the goal is active
  bool isGoalActive = true;
    if (goal->action == "start")
    {
      status = BT::NodeStatus::IDLE;
    std::cout << "Tree Started" << std::endl;

      // Loop until the goal is complete or preempted
      while (ros::ok() && isGoalActive && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
      {
        // Check for a new goal
        if (isNewGoalAvailable_)
        {
          // Check if the new goal is the same as the current goal
          if (current_goal_.action == goal->action)
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

        // Check if the goal has been preempted
        if (as_.isPreemptRequested())
        {
          // Indicate that the goal was preempted
          as_.setPreempted();
          // Set the flag indicating that the goal is no longer active
          isGoalActive = false;
          tree_.haltTree();
          success=false;
          break; 
        }
      }
      if (status==NodeStatus::FAILURE)
      {
        success=false;
      }
    }

  if (goal->action == "step")
  {
    tree_.tickRoot();
    std::cout << "Tree took a step" << std::endl;

    success=true;
  }
  if (goal->action == "stop")
  {
    std::cout << "Tree Stopped" << std::endl;
    tree_.haltTree();
    success=true;
  }

  if(success)
  {
    as_.setSucceeded();
  }
  if(!success)
  {
    as_.setAborted();
  }
} 
};

#endif