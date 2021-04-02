/**
* @file CSVtoPath.h
* @brief Flexible pose calculating class
* @author Shunya Hara
* @date 2021.2.26
* @details WayPointData(wpdata.csv) to nav_msgs/Path and Vector
*/

#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32MultiArray.h>


class CSVtoPath{
    private:
    std::vector<std::vector<std::string> > Score;
    double geometry_quat_getyaw(geometry_msgs::Quaternion geometry_quat);
    public:
    CSVtoPath(std::string& st,nav_msgs::Path& path,std_msgs::Int32MultiArray& type,std::string& map_frame);
     
};




CSVtoPath::CSVtoPath(std::string& st,nav_msgs::Path& path,std_msgs::Int32MultiArray& type,std::string& map_frame):Score(1,std::vector<std::string>(1)){
    using namespace std;
    ifstream ifs(st.c_str());
    // 開かなかったらエラー
    if (!ifs){
        ROS_ERROR("Error! File can not be opened\n");
    }
    string str = "";
    int i = 0;  // Score[i][ ]のカウンタ。
    int j = 0;  // Score[ ][j]のカウンタ。

    // ファイルの中身を一行ずつ読み取る
    while(getline(ifs, str))
    {
        string tmp = "";
        istringstream stream(str);
 
        // 区切り文字がなくなるまで文字を区切っていく
        while (getline(stream, tmp, ','))
        {
            Score.at(i).push_back(tmp);
            j++;
        }
        j = 0;
        i++;  
        Score.resize(i+1);
    }
     //先頭行のタグを消す
     Score.erase(Score.begin());
     //Score.resize(Score.size()-2);
    //サイズをwpの大きさに変更
    //path.poses.resize(Score.size()-1);
    //csvの値をPathとVectorに代入
    for(int k=0;k<Score.size()-1;k++){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x=atof(Score.at(k).at(1).c_str());
        pose.pose.position.y=atof(Score.at(k).at(2).c_str());
        pose.pose.position.z=atof(Score.at(k).at(3).c_str());
        pose.pose.orientation.x=atof(Score.at(k).at(4).c_str());
        pose.pose.orientation.y=atof(Score.at(k).at(5).c_str());
        pose.pose.orientation.z=atof(Score.at(k).at(6).c_str());
        pose.pose.orientation.w=atof(Score.at(k).at(7).c_str());
        pose.header.frame_id=map_frame;
        pose.header.stamp=ros::Time::now();
        type.data.push_back(atof(Score.at(k).at(8).c_str()));
        path.poses.push_back(pose);
     }
     path.header.frame_id=map_frame;
     path.header.stamp=ros::Time::now();
     ifs.close();

}
/*
void csvread2::print(){
     
    for(int k=0;k<Score.size();k++){
    	for(int l=0;l<Score.at(k).size();l++){
    	    cout <<Score.at(k).at(l).c_str()<<",";
    	}
    	cout <<endl;
    }
}

double csvread2::geometry_quat_getyaw(geometry_msgs::Quaternion geometry_quat){
    tf::Quaternion quat;
    double roll,pitch,yaw;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
    return yaw;
}

*/
