/**
* @file wpSelect.cpp
* @brief select target way point
* @author Michikuni Eguchi
* @date 2021.9.21
* @details 位置情報からpublishするtarget way pointとなるpose を選ぶ
*          (等間隔で設定されてない5〜10mの少し離れた間隔のwp pointをもとにする)
*/
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <string>
#include "waypoint_tools/TFtoPose.h"
#include "waypoint_tools/button_status.h"
#include "waypoint_tools/robot_status.h"

//poseStamp間の距離
double poseStampDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
    double diffX = pose1.pose.position.x - pose2.pose.position.x;
    double diffY = pose1.pose.position.y - pose2.pose.position.y;
    double diffZ = pose1.pose.position.z - pose2.pose.position.z;

    return sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
}

template<class T> T constrain(T num, double minVal, double maxVal)
{
    if(num > maxVal){
        num = maxVal;
    }
    if(num < minVal){
        num = minVal;
    }

    return num;
}

std_msgs::Int32 now_wp;
void set_wp_callback(const std_msgs::Int32& now_wp_){
    now_wp=now_wp_;
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path path_message)
{
    path = path_message;
}

bool isSuccessPlanning = true;
int failedPlanCount = 0;
void successPlan_callback(const std_msgs::Bool successPlan_message)
{
    isSuccessPlanning = successPlan_message.data;
}

int button_clicked=0;
void buttons_callback(const std_msgs::Int32 sub_buttons){
    button_clicked=sub_buttons.data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpControll");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string map_id, base_link_id;
    pnh.param<std::string>("map_frame_id", map_id, "map");
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    double target_deviation, fin_tar_deviation;
    pnh.param<double>("target_deviation", target_deviation, 0.5);
    pnh.param<double>("final_target_deviation", fin_tar_deviation, 0.1);
    double rate;
    pnh.param<double>("loop_rate", rate, 100);
    double maxVelocity;
    pnh.param<double>("maxVelocity", maxVelocity, 1.0);

    TFtoPose now_position(map_id, base_link_id, rate);

    ros::Subscriber set_wp_sub = nh.subscribe("waypoint/set", 50, set_wp_callback);
    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Subscriber successPlan_sub = nh.subscribe("successPlan", 10, successPlan_callback);
    //rviz control panel subscliber
    ros::Subscriber buttons_sub = nh.subscribe("buttons", 50, buttons_callback);
    ros::Publisher nowWp_pub = nh.advertise<std_msgs::Int32>("waypoint/now", 10);
    ros::Publisher nowPos_pub = nh.advertise<geometry_msgs::PoseStamped>("nowWpPose", 10);
    ros::Publisher mode_pub = nh.advertise<std_msgs::String>("mode_select/mode", 10);

    ros::Rate loop_rate(rate);

    bool trace_wp_mode = true;

    std_msgs::String mode;
    mode.data =  robot_status_str(robot_status::angleAdjust);

    bool isReach = false;

    now_wp.data = 0;

    geometry_msgs::PoseStamped nowPos;
    while(ros::ok())
    {
        if(path.poses.size()>0){
            if(trace_wp_mode){
                //target_deviationになるよう target way pointの更新
                while(!(poseStampDistance(path.poses[now_wp.data], now_position.toPoseStamped()) >= target_deviation))
                {
                    //end point
                    if(now_wp.data >= (path.poses.size()-1)){
                        break;
                    }
                    now_wp.data++;
                }
            }

            //if planning fail, increase the target deviation
            double fin_tar_deviation_;
            if(isSuccessPlanning){
                fin_tar_deviation_ = fin_tar_deviation;
            }else{
                fin_tar_deviation_ = 2 * fin_tar_deviation;
            }

            if(now_wp.data >= (path.poses.size()-1)){
                //distance
                if(!isReach){
                    //reach last wp point
                    if(poseStampDistance(path.poses[now_wp.data], now_position.toPoseStamped()) <= fin_tar_deviation_){
                        isReach = true;
                        mode_pub.publish(mode);
                    }
                }
            }

            //restart running
            if(button_clicked==buttons_status_start){
                isReach = false;
            }

            nowPos.header.frame_id = path.header.frame_id;
            nowPos.header.stamp = ros::Time::now();
            nowPos.pose = path.poses[now_wp.data].pose;

            nowPos_pub.publish(nowPos);
            nowWp_pub.publish(now_wp);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}