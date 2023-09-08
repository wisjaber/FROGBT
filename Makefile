# individual nodes 

.PHONY: arm detections skills tag_check servers simulation tree_server bt_execution drop reset_onto setup_scenario1 scenario1 rbt rtree list_bt frogbt

arm:
	rosrun airlab move_arm_server.py

detections:
	roslaunch albert_skills apriltag_detection.launch --wait 

skills:
	roslaunch albert_skills load_skills_moveit.launch --wait 

tag_check:
	rosrun apriltag_filter apriltag_filter_single_detection_server_node 

gripper_check:
	rosrun bt_tests vacuum_state_service_server.py
	
tree_server:
	rosrun actionlib_tools axclient.py /btserver

simulation: 
	roslaunch albert_gazebo albert_gazebo_navigation.launch panda_control_mode:=moveit localization:=amcl world:=AH_store initial_pose_x:=-2.3

bt_execution:
	rosrun bt_tests genbt

setup_scenario1:
	@rosrun bt_tests servers_node.py & \
	rosrun bt_tests Tgenbt --log

scenario1:
	rosrun bt_tests tutorial_bt.py

rbt:
	rosrun bt_tests reason_construct.py

rtree:
	rosrun bt_tests recoverymake.py

drop:
	rosrun actionlib_tools axclient.py /franka_vacuum_gripper/dropoff

frogbt:
	rosrun bt_tests frogbt.py
	
list_bt:
	rosrun bt_tests prepare_list1.py

servers:
	@rosrun bt_tests servers_node_sim.py & \
	rosrun apriltag_filter apriltag_filter_single_detection_server_node & \
	rosrun airlab VacuumCheck_server.py & \
	rosrun airlab PoseCheck_server.py & \
	rosrun airlab HomeCheck_server.py & \
	rosrun airlab arm_home_actionserver.py & \
	rosrun airlab LocationCheck_server.py & \
	rosrun airlab DeliverCheck_server.py & \
	rosrun airlab pick_from_floor.py & \
	rosrun airlab scan_floor.py & \
	rosrun airlab approach_actionserver.py & \
	roslaunch albert_skills apriltag_detection.launch --log & \
	roslaunch albert_skills load_skills_moveit.launch --log

reset_onto:
	python3 ./scripts/reset_onto.py

