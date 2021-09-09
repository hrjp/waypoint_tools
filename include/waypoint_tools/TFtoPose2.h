
/**
* @file TFtoPose.cpp
* @brief TF to Pose,PoseStamped and TransformStamped
* @author Shunya Hara
* @date 2021.3.7
* @details TFを定期的に取得してPose,PoseStamped,TransformStampedに変換する
*/

#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <turtlesim/Spawn.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf2/transform_datatypes.h>



class TFtoPose{
public:

    TFtoPose(std::string& base_id, std::string& child_id, double rate=10.0);

    geometry_msgs::Pose toPose();
    geometry_msgs::PoseStamped toPoseStamped();
    geometry_msgs::TransformStamped toTransformStamped();

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    std::string base_id_,child_id_;
    geometry_msgs::TransformStamped tfstamped;
    geometry_msgs::PoseStamped posestamped;

    tf::TransformListener listener;
    tf::StampedTransform trans_slam;
};




TFtoPose::TFtoPose(std::string& base_id, std::string& child_id, double rate) : nh_(), tfBuffer_(), tfListener_(tfBuffer_){
    base_id_=base_id;
    child_id_=child_id;
    double duration = 1.0/rate;
    timer_ = nh_.createTimer(ros::Duration(duration), [&](const ros::TimerEvent& e) {
      /*
      try
      {
        tfstamped = tfBuffer_.lookupTransform(base_id, child_id, ros::Time(0));
        posestamped.header=tfstamped.header;
        posestamped.header.frame_id=base_id;
        posestamped.pose.position.x=tfstamped.transform.translation.x;
        posestamped.pose.position.y=tfstamped.transform.translation.y;
        posestamped.pose.position.z=tfstamped.transform.translation.z;
        posestamped.pose.orientation=tfstamped.transform.rotation;
      }

      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        return;
      } */   

    try {
        listener.lookupTransform(base_id, child_id,ros::Time(0), trans_slam);
        posestamped.pose.position.x = trans_slam.getOrigin().x();
        posestamped.pose.position.y = trans_slam.getOrigin().y();
        posestamped.pose.position.z = trans_slam.getOrigin().z();
        posestamped.pose.orientation.x=trans_slam.getRotation().getX();
        posestamped.pose.orientation.y=trans_slam.getRotation().getY();
        posestamped.pose.orientation.z=trans_slam.getRotation().getZ();
        posestamped.pose.orientation.w=trans_slam.getRotation().getW();
        //ROS_INFO("X:%f , Y:%f , id:%s",posestamped.pose.position.x,posestamped.pose.position.y,base_id);
        std::cout<<base_id<<std::endl;
    }
    catch (tf::TransformException &ex)  {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        
    }


      
    });

}

geometry_msgs::Pose TFtoPose::toPose(){
    return posestamped.pose;
}

geometry_msgs::PoseStamped TFtoPose::toPoseStamped(){
    return posestamped;
}

geometry_msgs::TransformStamped TFtoPose::toTransformStamped(){
    return tfstamped;
}



/*

class tf_lis{
    public:
    tf_lis(const char *base_id,const char *child_id);
    Vector update();
    Vector pos;
    private:
    ros::NodeHandle n;
    tf::TransformListener listener;
    string tf_name1;
    string tf_name2;
    tf::StampedTransform trans_slam;
};

tf_lis::tf_lis(const char *base_id,const char *child_id):listener(ros::Duration(10)){
    ros::NodeHandle private_nh("~");
    private_nh.param("tf_name1",tf_name1,std::string(child_id));
    private_nh.param("tf_name2",tf_name2,std::string(base_id));

}

Vector tf_lis::update(){
    
     try {
        listener.lookupTransform(tf_name2, tf_name1,ros::Time(0), trans_slam);
        pos.x = trans_slam.getOrigin().x();
        pos.y= trans_slam.getOrigin().y();
        pos.yaw = tf::getYaw(trans_slam.getRotation());
    }
    catch (tf::TransformException &ex)  {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        
    }
    return pos;
}
*/