# individual nodes 

.PHONY: arm detections skills tag_check stoef atlas tree tree_server bankai drop reset kitting rbt

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

tree:
	rosrun bt_tests first_tree
	
tree_server:
	rosrun actionlib_tools axclient.py /btserver

atlas: 
	roslaunch albert_gazebo albert_gazebo_navigation.launch panda_control_mode:=moveit localization:=amcl world:=AH_store initial_pose_x:=-1.4

bankai:
	rosrun bt_tests genbt
rbt:
	rosrun bt_tests RBT

drop:
	rosrun actionlib_tools axclient.py /franka_vacuum_gripper/dropoff

stoef:
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

reset:
	rosrun bt_tests reset_onto.py

kitting:
	rosrun bt_tests servers_node.py
