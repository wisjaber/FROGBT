# individual nodes 

fix_arm:
	rosrun bt_tests move_arm_server.py

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

run_stoef:
	@rosrun apriltag_filter apriltag_filter_single_detection_server_node & \
	rosrun bt_tests VacuumCheck_server.py & \
	rosrun bt_tests PoseCheck_server.py & \
	rosrun bt_tests HomeCheck_server.py & \
	rosrun bt_tests arm_home_actionserver.py & \
	rosrun bt_tests LocationCheck_server.py & \
	roslaunch albert_skills apriltag_detection.launch --log & \
	roslaunch albert_skills load_skills_moveit.launch --log

.PHONY: fix_arm detections skills tag_check run_stoef atlas tree tree_server bankai