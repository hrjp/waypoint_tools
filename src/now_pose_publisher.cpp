/**
* @file now_pose_publisher.cpp
* @brief now amcl_pose -> initilapose 
* @author Shunya Hara
* @date 2021.10.21
* @details このノードを起動すると一度だけ現在の自己位置(amcl_pose)をinitialposeとして配信する(rosbagでのdebug用)
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

using namespace std;

ros::Publisher initialpose_pub;
bool pub_once=false;

void nowpose_callback(const geometry_msgs::PoseWithCovarianceStamped& nowpose){
    initialpose_pub.publish(nowpose);
    ROS_DEBUG("initialpose published once");
    pub_once=true;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "now_pose_publisher");
    ros::NodeHandle n;
    //制御周期10Hz
    ros::Rate loop_rate(10);

    //param setting
    ros::NodeHandle pn("~");

    ros::NodeHandle lSubscriber("");

    //amcl_pose subscliber
    ros::Subscriber mcl_cmd_vel_sub = lSubscriber.subscribe("amcl_pose", 50, nowpose_callback);

    //pose publisher
    initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    while (n.ok())  {
        
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        if(pub_once){
            break;
        }
        
    }
    
    return 0;
}
