/**
* @file FlexPose.h
* @brief Flexible pose calculating class
* @author Shunya Hara
* @date 2021.2.26
* @details 
*/
//wpをcsvから読み込んでpathとmarkerarrayで配信する
//waypoint/nowにあわせてmarkerの色を変える

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "CSVtoPath.h"

using namespace std;

int main(int argc, char **argv){
    
    ros::init(argc, argv, "wp_loader_node");
    ros::NodeHandle n;
    //制御周期10Hz
    ros::Rate loop_rate(10);

    //ウェイポイントファイルのロード
    ros::NodeHandle pn("~");
    string filename;
    pn.getParam("waypointfile",filename);

    nav_msgs::Path path;
    CSVtoPath csv(filename,path);
    
    //Path publisher
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("waypoint/path", 1);

    while (n.ok())  {
        path_pub.publish(path);
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}

