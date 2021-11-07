#!/usr/bin/env python

import pandas as pd
import rospy
import rosparam
import csv
import sys
from move_base_msgs.msg import MoveBaseActionGoal
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32
from nav_msgs.msg import Path

i=0
map_num=0
path=Path

def mapcallback(map_num_row):
    global map_num
    map_num=map_num_row.data

def pathcallback(path_row):
    global path
    path=path_row

def navcallback(nav_row):
    lon.data.apped(nav_row.longtitude);
    alt.data.apped(nav_row.altitude)


def listener():
    global path
    rospy.init_node('waypoint_manager', anonymous=True)
    wp_dir=rospy.get_param('~wp_dir')
    predata=pd.read_csv(wp_dir+'wpdata.csv')
    # get timestamp
    from datetime import datetime as dt
    tdatetime = dt.now()
    tstr = tdatetime.strftime('%y%m%d_%H%M%S')
    # output to excel file
    predata.drop(predata.columns[[0]], axis=1,inplace=True)
    predata.to_csv(wp_dir+'wpdata'+tstr+".csv")

    df=pd.DataFrame(columns=['x', 'y','z','qx','qy','qz','qw','type','map'])

    rospy.Subscriber("/map_num", Int32, mapcallback)
    rospy.Subscriber("waypoint/path",Path,pathcallback)

    rospy.spin()

    i=0
    for j in path.poses:
        df.loc[i] = [j.pose.position.x,j.pose.position.y,j.pose.position.z,j.pose.orientation.x,j.pose.orientation.y,j.pose.orientation.z,j.pose.orientation.w,1,1]
        df.to_csv(wp_dir+'wpdata.csv', header=True)
        i+=1
    print(" WayPoint SAVED!!")

if __name__ == '__main__':
    listener()
