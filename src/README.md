# waypoint_tools

## 3d_initial_pose
2D Pose Estimate を3Dに拡張するノード
### publisher
* /initialpose [geometry_msgs::PoseWithCovarianceStamped]
* /initialpose_candidate [geometry_msgs::PoseStamped]
### subscriber
* /move_base_simple/goal[geometry_msgs::PoseStamped]
* /initialpose_z [std_msgs::Float32]
* buttons [std_msgs::Int32]
### parameter
none


## path_number
pathを読んでwaypointの番号をmarkerarrayで配信する可視化用ノード  
waypoint/nowにあわせてmarkerの色を変える
### publisher
* waypoint/marker [visualization_msgs::MarkerArray]
### subscriber
* waypoint/now [std_msgs::Int32]
* waypoint/path [nav_msgs::Path]
### parameter
none


## wp_control
pathを受けっとってそれをなぞって走行する
### publisher
* waypoint/set [std_msgs::Int32]
* /initialpose [geometry_msgs::PoseWithCovarianceStamped]
### subscriber
* waypoint/now [std_msgs::Int32]
* waypoint/path [nav_msgs::Path]
* buttons [std_msgs::Int32]
### parameter
none


## wp_loader
wpをcsvから読み込んでpathとmarkerarrayで配信する  
waypoint/nowにあわせてmarkerの色を変える
### publisher
* waypoint/path [nav_msgs::Path]
* waypoint/type [std_msgs::Int32MultiArray]
### subscriber
none
### parameters
* map_frame [string] (default : map)


## wp_tracer
startボタンがおされたらmap->base_linkの距離を監視して決められた距離動くたびにnav_msgs/Pathに記録する
### publisher
* waypoint/path [nav_msgs::Path]
* waypoint/now [std_msgs::Int32]
### subscriber
* buttons [std_msgs::Int32]
### parameter
* map_frame_id [string] (default : map)
* base_link_frame_id [string] (default : base_link)
* waypoint_pitch [double] (default : 1.0)
* loop_rate [double] (default : 10.0)
