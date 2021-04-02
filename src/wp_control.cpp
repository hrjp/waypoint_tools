/**
* @file wp_control.cpp
* @brief 
* @author Shunya Hara
* @date 2021.3.11
* @details pathを受けっとってそれをなぞって走行する
*/


#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Path.h>

int int_constrain(int val,int down_limit,int up_limit){
  if(val>up_limit){
    return up_limit;
  }
  if(val<down_limit){
    return down_limit;
  }
  return val;
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path& path_){
    path=path_;
}


int now_wp;
void now_wp_callback(const std_msgs::Int32 now_wp_){
    now_wp=now_wp_.data;
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

int button_clicked=0;
void buttons_callback(const std_msgs::Int32 sub_buttons){
    button_clicked=sub_buttons.data;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "initial_pose_3d");
    ros::NodeHandle n;
    //制御周期10Hz
    ros::Rate loop_rate(10);

    ros::NodeHandle lSubscriber("");

    //waypoint/now subscliber
    ros::Subscriber now_wp_sub = lSubscriber.subscribe("waypoint/now", 50, now_wp_callback);
    //waypoint/path subscliber
    ros::Subscriber path_sub = lSubscriber.subscribe("waypoint/path", 50, path_callback);
    //rviz control panel subscliber
    ros::Subscriber buttons_sub = lSubscriber.subscribe("buttons", 50, buttons_callback);

    //wp set publisher
    ros::Publisher wp_pub=n.advertise<std_msgs::Int32>("waypoint/set", 1);
    //2D_POSE_ESTIMATE publisher
    ros::Publisher initial_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    geometry_msgs::PoseWithCovarianceStamped pubpose;
    //3D pose の共分散
    pubpose.pose.covariance[0]=0.25;
    pubpose.pose.covariance[7]=0.25;
    pubpose.pose.covariance[14]=0.25/2.0;
    pubpose.pose.covariance[21]=0.06853891945200942/4.0;
    pubpose.pose.covariance[28]=0.06853891945200942/4.0;
    pubpose.pose.covariance[35]=0.06853891945200942;

    while (n.ok())  {

        if(button_clicked==buttons_status_initialpose){
            pubpose.header=path.header;
            pubpose.pose.pose=path.poses.at(now_wp).pose;
            initial_pub.publish(pubpose);
        }
        if(button_clicked==buttons_status_mynus){
            std_msgs::Int32 msg;
            msg.data=int_constrain(now_wp-1,-1,path.poses.size());
            now_wp=int_constrain(now_wp-1,-1,path.poses.size());
            wp_pub.publish(msg);
        }
        if(button_clicked==buttons_status_plus){
            std_msgs::Int32 msg;
            msg.data=int_constrain(now_wp+1,-1,path.poses.size());
            now_wp=int_constrain(now_wp+1,-1,path.poses.size());
            wp_pub.publish(msg);
        }
        button_clicked=buttons_status_free;
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}
