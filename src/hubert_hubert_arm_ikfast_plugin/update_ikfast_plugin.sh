search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=hubert.srdf
robot_name_in_srdf=hubert
moveit_config_pkg=hubert_moveit_config
robot_name=hubert
planning_group_name=hubert_grp
ikfast_plugin_pkg=hubert_hubert_grp_ikfast_plugin
base_link_name=base_link
eef_link_name=end_effector
ikfast_output_path=/home/damodar/hubert_ws/src/hubert_hubert_grp_ikfast_plugin/src/hubert_hubert_grp_ikfast_solver.cpp
eef_direction="0 0 1"

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  --eef_direction $eef_direction\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
