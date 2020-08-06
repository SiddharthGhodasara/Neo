search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=mpo_500.srdf
robot_name_in_srdf=mpo_500
moveit_config_pkg=mpo_500_moveit_config
robot_name=mpo_500
planning_group_name=l_arm
ikfast_plugin_pkg=mpo_500_l_arm_ikfast_plugin
base_link_name=ur5_left_base_link
eef_link_name=ur5_left_tool0
ikfast_output_path=/home/siddharth/ws_moveit/src/neo_simulation/robots/mpo_500/mpo_500_l_arm_ikfast_plugin/src/mpo_500_l_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
