/**
* @file path_number.cpp
* @brief path number node for Rviz
* @author Shunya Hara
* @date 2021.3.5
* @details pathを読んでwaypointの番号をmarkerarrayで配信する可視化用ノード
*          waypoint/nowにあわせてmarkerの色を変える
*/


#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>


using namespace std;

//color setting
std_msgs::ColorRGBA setColor(double r,double g,double b,double a){
    std_msgs::ColorRGBA setcolor;
    setcolor.r=r;
    setcolor.g=g;
    setcolor.b=b;
    setcolor.a=a;
    return setcolor;
}

std_msgs::ColorRGBA black_color=setColor(0.0,0.0,0.0,1.0);
std_msgs::ColorRGBA gray_color=setColor(0.3,0.3,0.3,1.0);
std_msgs::ColorRGBA blue_color=setColor(0.0,0.0,1.0,1.0);

int now_wp=0;
void now_wp_callback(const std_msgs::Int32& now_wp_){
    now_wp=now_wp_.data;
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path& path_){
    path=path_;
}

//pathからwaypoint numberのマーカーを生成
visualization_msgs::MarkerArray generatePathNumber(const nav_msgs::Path& path,int now_wp){
    //number text size
    const double text_size=1.0;
    //number z clearance
    const double clearamce_z=1.0;
    visualization_msgs::MarkerArray marker_array;
    for(int i=0;i<path.poses.size();i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id=path.header.frame_id;
        marker.header.stamp=ros::Time::now();
        marker.ns = "waypoint_marker";
        marker.id=i;
        marker.lifetime = ros::Duration();
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x=text_size;
        marker.scale.y=text_size;
        marker.scale.z=text_size;
        marker.pose=path.poses.at(i).pose;
        marker.pose.position.z+=clearamce_z;
        ostringstream oss;
        oss<<i;
        marker.text= oss.str().c_str();
        if(now_wp<i){marker.color=black_color;}
        if(now_wp==i){marker.color=blue_color;}
        if(now_wp>i){marker.color=gray_color;}
        
        marker_array.markers.push_back(marker);
    }
    return marker_array;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "path_number");
    ros::NodeHandle n;
    //制御周期10Hz
    ros::Rate loop_rate(10);

    //param setting
    ros::NodeHandle pn("~");   
    
    //Marker publisher
    ros::Publisher marker_pub=n.advertise<visualization_msgs::MarkerArray>("waypoint/marker", 1);
    
    ros::NodeHandle lSubscriber("");

    //waypoint/now subscliber
    ros::Subscriber now_wp_sub = lSubscriber.subscribe("waypoint/now", 50, now_wp_callback);
    //waypoint/path subscliber
    ros::Subscriber path_sub = lSubscriber.subscribe("waypoint/path", 50, path_callback);

    while (n.ok())  {

        visualization_msgs::MarkerArray marker_array=generatePathNumber(path,now_wp);

        marker_pub.publish(marker_array);

        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}

