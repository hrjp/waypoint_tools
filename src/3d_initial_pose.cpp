/**
* @file 3d_initial_pose.cpp
* @brief 2D Pose Estimate expands 3D
* @author Shunya Hara
* @date 2021.3.5
* @details 2D Pose Estimate を3Dに拡張するノード
*/

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseWithCovariance.h>

geometry_msgs::PoseWithCovarianceStamped pubpose;
double zpos;

ros::Publisher candidate_pub;
ros::Publisher initial_pub;


void pose2d_callback(const geometry_msgs::PoseStamped& pose2d){
    geometry_msgs::PoseStamped pos=pose2d;
    pos.pose.position.z=zpos;

    pubpose.header=pos.header;
    pubpose.pose.pose=pos.pose;

    candidate_pub.publish(pos);

}

void zpos_callback(const std_msgs::Float32& zpos_){
    pubpose.header.stamp=ros::Time::now();
    zpos=zpos_.data;
    pubpose.pose.pose.position.z=zpos;

    geometry_msgs::PoseStamped pos;
    pos.header=pubpose.header;
    pos.pose=pubpose.pose.pose;
    candidate_pub.publish(pos);

}

enum buttons_status{
    buttons_status_free,//0
    buttons_status_start,//1
    buttons_status_pause,//2
    buttons_status_initialpose,//3
    buttons_status_plus,//4
    buttons_status_mynus,//5
    buttons_status_reset,//6
    buttons_status_zpublish//7
  };
void buttons_callback(const std_msgs::Int32& buttons){
    if(buttons.data==buttons_status_zpublish){
        initial_pub.publish(pubpose);
    }
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "initial_pose_3d");
    ros::NodeHandle n;
    //制御周期10Hz
    //ros::Rate loop_rate(10);

    ros::NodeHandle lSubscriber("");
    //set way point subscliber

    //2d nav goal subscliber
    ros::Subscriber pose2d_sub = lSubscriber.subscribe("/move_base_simple/goal", 50, pose2d_callback);

    //z position subscliber
    ros::Subscriber zpos_sub = lSubscriber.subscribe("/initialpose_z", 50, zpos_callback);

    //2D_POSE_ESTIMATE publisher
    initial_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    //pose publisher
    candidate_pub = n.advertise<geometry_msgs::PoseStamped>("/initialpose_candidate", 1);

    //rviz control panel subscliber
    ros::Subscriber buttons_sub = lSubscriber.subscribe("buttons", 50, buttons_callback);

    //3D pose の共分散
    pubpose.pose.covariance[0]=0.25*10.0;
    pubpose.pose.covariance[7]=0.25*10.0;
    pubpose.pose.covariance[14]=0.25*2.0;
    pubpose.pose.covariance[21]=0.0*0.06853891945200942/4.0;
    pubpose.pose.covariance[28]=0.0*0.06853891945200942/4.0;
    pubpose.pose.covariance[35]=0.0;//6853891945200942/3.0;
    int a;

    ros::spin();
    
    return 0;
}
