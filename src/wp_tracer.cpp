/**
* @file wp_tracer.cpp
* @brief way point tracer
* @author Shunya Hara
* @date 2021.3.7
* @details startボタンがおされたらmap->base_linkの距離を監視して決められた距離動くたびにnav_msgs/Pathに記録する
*/

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <waypoint_tools/TFtoPose.h>
#include <waypoint_tools/FlexPose.h>
#include <waypoint_tools/button_status.h>

using namespace std;

int button_status=0;
void buttons_callback(const std_msgs::Int32& buttons){
    button_status=buttons.data;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "wp_tracer");
    ros::NodeHandle n;
    

    //param setting
    ros::NodeHandle pn("~");
    string map_id,baselink_id;
    double wp_pitch=1.0;
    double looprate=10.0;
    pn.param<string>("map_frame_id",map_id,"map");
    pn.param<string>("base_link_frame_id",baselink_id,"base_link");
    pn.param<double>("waypoint_pitch",wp_pitch,1.0);
    pn.param<double>("loop_rate",looprate,10.0);

    //制御周期10Hz
    ros::Rate loop_rate(looprate);

    //map->base_link のTFを取得
    TFtoPose now_position(map_id,baselink_id,looprate);

    ros::NodeHandle lSubscriber("");

    //rviz control panel subscliber
    ros::Subscriber buttons_sub = lSubscriber.subscribe("buttons", 50, buttons_callback);

    //WayPointPath　publisher
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("waypoint/path", 10);

    //Now WayPoint publisher
    ros::Publisher nowwp_pub = n.advertise<std_msgs::Int32>("waypoint/now", 10);

    nav_msgs::Path path;
    FlexPose prepos(0.0,0.0);
    bool trace_mode=false;
    
    while (n.ok())  {

        auto nowpos=FlexPose(now_position.toPoseStamped());
        path.header=nowpos.toPoseStamped().header;
        //waypointの記録をスタート
        if(button_status==buttons_status_start){
            trace_mode=true;
        }
        //waypointの記録を止める
        if(button_status==buttons_status_pause){
            trace_mode=false;
        }

        //一定距離進むごとにwaypointを記録
        if(trace_mode){
            if((nowpos-prepos).size()>wp_pitch){
                prepos=nowpos;
                path.poses.push_back(nowpos.toPoseStamped());
            }
        }

        //最新のwaypointを手動で記録
        if(button_status==buttons_status_plus){
            prepos=nowpos;
            path.poses.push_back(nowpos.toPoseStamped());
        }
        //最新のwaypointを消す
        if(button_status==buttons_status_mynus){
            if(path.poses.size()>0){
                prepos=nowpos;
                path.poses.pop_back();
            }
        }

        std_msgs::Int32 nowwp;
        nowwp.data=path.poses.size()-1;
        nowwp_pub.publish(nowwp);
        path.header.frame_id=map_id;
        path_pub.publish(path);
        
        button_status=buttons_status_free;
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}