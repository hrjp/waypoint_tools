/**
* @file waypoint_editor_selector.cpp
* @brief node
* @author Shunya Hara
* @date 2021.mm.dd
* @details 
*/

#include <ros/ros.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

using namespace std;

int main(int argc, char **argv){
    
    ros::init(argc, argv, "wp_loader_node");
    ros::NodeHandle n;
    //制御周期10Hz
    ros::Rate loop_rate(10);

    //param setting
    ros::NodeHandle pn("~");

    while (n.ok())  {
        
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}
