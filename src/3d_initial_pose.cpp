/**
* @file 3d_initial_pose.cpp
* @brief 2D Pose Estimate expands 3D
* @author Shunya Hara
* @date 2021.3.5
* @details 2D Pose Estimate を3Dに拡張するノード
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>

void pose2d_callback(const geometry_msgs::PoseStamped& pose2d){
    
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "3d_initial_pose");
    ros::NodeHandle n;
    //制御周期10Hz
    ros::Rate loop_rate(10);

    ros::NodeHandle lSubscriber("");
    //set way point subscliber

    //2d nav goal subscliber
    ros::Subscriber pose2d_sub = lSubscriber.subscribe("/move_base_simple/goal", 50, pose2d_callback);

    //2D_POSE_ESTIMATE publisher
    ros::Publisher initial_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    while (n.ok())  {

        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}
