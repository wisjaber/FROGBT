# Failure Recovery with Ontologically Generated Behaviour Trees (FROGBT)
Welcome to the Failure Recovery with Ontologically Generated Behaviour Trees (FROGBT) project repository! This project is part of the Master's program in Robotics at the Technical University of Delft and represents a significant step forward in the field of robotics, specifically in the domain of behaviour trees and failure recovery.

## Introduction
FROGBT represents a novel fusion of behaviour trees (BTs), a hierarchical task execution method, and ontological reasoning, a framework for structuring knowledge. By seamlessly integrating ontological reasoning into the process of BT generation, FROGBT aims to bridge the gap between generated BTs and robust failure recovery strategies. This integration imbues BT generation with contextual awareness, enabling robots to make informed decisions in complex and dynamic environments.

## Getting started
- To be able to run FROGBT tutorials you need to have a ROS noetic installation: [noetic installation instructions](http://wiki.ros.org/noetic/Installation/Ubuntu)
- You need to have the following modules installed: [owlready2](https://pypi.org/project/owlready2/) and [lxml](https://pypi.org/project/lxml/)
```bash
pip install owlready2
pip install lxml
```
- You also need the [BehaviorTreeCPP](https://github.com/BehaviorTree/BehaviorTree.CPP) package **v3.8** which can be installed using the following line 
```bash
  sudo apt-get install ros-$ROS_DISTRO-behaviortree-cpp-v3
```
- Clone [BehaviorTrees.ROS](https://github.com/BehaviorTree/BehaviorTree.ROS) repo, a useful package to use actions and services of ROS in BehaviorTreesCPP package. 
```bash
git clone https://github.com/BehaviorTree/BehaviorTree.ROS.git
```
- For visualization you will need GROOT [installation instruction](https://github.com/wisjaber/Groot) 
	- GROOT proved somewhat challenging to get working with v3.8 of BehaviorTreeCPP package, thus a fork was made with changes that made it work for my setup. 		
	```bash
	git clone --branch wisjaber-patch-1 https://github.com/wisjaber/Groot.git
	```
Since GROOT is only for visualization, if it doesn't work you can comment out the following line in the "bt_generation_action.h" and "tutorial_bt_generation_action.h" files
	```bash
		PublisherZMQ publisher_zmq(tree_,200);
	```
- For the skills and simulated environment you need to request access to the gits from [AIRLab](https://icai.ai/airlab-delft/) 
- Clone the [Gazebo assets](https://github.com/tud-airlab/airlab_gazebo_assets) repo. (don't forget to source the environment variables as it says in the github)
	```bash
	git clone https://github.com/tud-airlab/airlab_gazebo_assets.git
	```
- Clone [Albert skills](https://github.com/tud-airlab/albert_skills) repo
	```bash 
	git clone https://github.com/tud-airlab/albert_skills.git
	```
- Clone [Albert](https://github.com/tud-airlab/albert) repo
	```bash
	git clone https://github.com/tud-airlab/albert.git
```

- Clone the msgs package 
```bash
git clone https://github.com/wisjaber/FROGBT_msgs.git
```
- Clone the extra skills for recovery package
```bash
git clone https://github.com/wisjaber/airlab_extra_skills.git
```
- Clone the main repo
```bash
git clone https://github.com/wisjaber/FROGBT.git
```

## How To Use
There are three scenarios to evaluate FROGBT you can run each following the instructions below 
### Scenario one
This scenario is comparison with [SkiROS2](https://github.com/RVMI/skiros2) framework in their example task found in [SkiROS examples](https://github.com/RVMI/skiros2_examples)
In order to run the scenario using FROGBT build your workspace and source it. 
- Then in a sourced terminal
```bash
cd workspace/src/FROGBT
make setup_scenario1
```
- Start GROOT, if you have it, and choose monitor
![[Pasted image 20230908110601.png]]
- In another sourced terminal run
```bash
make scenario1
```
- Immediately connect to GROOT to see the BT growing. Alternatively, you can see the tree execution in the first terminal
![[Pasted image 20230908110755.png]]
### Scenario Two
This scenario is preparing a list of products in the simulated environment. There are two different ways to evaluate it: Sequence List, and BT Reuse. 
#### Sequence List
- To start the simulation in a sourced terminal run 
```bash
cd workspace/src/FROGBT
make simulation
```
- sometimes the arm gets into singularity when loading the simulation ![[Pasted image 20230908130020.png]]If that happened, run the following in a terminal
	```bash
make arm
```
- Run the servers in a sourced terminal by running
```bash
make servers
```
- Start the server for the BT execution a sourced terminal by running 
```bash
make bt_execution
```
- Start your GROOT and in a terminal run the following to start the list of products tree generation
```bash
make list_bt
```
#### BT Reuse
Follow the same steps from the Sequence List instruction behalf the last one. instead run the following
```bash
make frogbt
```
This will run the frogbt main script to place one product, tag=1, thus generating a BT for it. After it is done to reuse that BT run the following
```bash
rosrun bt_tests bt_reuse.py place milk <tag>
```
Replace <tag> with the number of the product you want, in the ontology there are 4 products defined with the tags: 1,18,19,20. 
you can reuse it for a different product multiple times until there are no products defined in the ontology.


