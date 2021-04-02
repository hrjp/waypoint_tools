#!/usr/bin/env python
import pandas as pd
import rospy
import csv
import sys
from move_base_msgs.msg import MoveBaseActionGoal
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32
#from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
#from sensor_msgs import NavSatFix
#from visualization_msgs.msg import Marker1

predata=pd.read_csv('~/catkin_ws/src/kcctcore/config/waypointdata/wpdata.csv')
# get timestamp
from datetime import datetime as dt
tdatetime = dt.now()
tstr = tdatetime.strftime('%y%m%d_%H%M%S')
# output to excel file
predata.drop(predata.columns[[0]], axis=1,inplace=True)
predata.to_csv('~/catkin_ws/src/kcctcore/config/waypointdata/wpdata'+tstr+".csv")

df=pd.DataFrame(columns=['x', 'y','z','qx','qy','qz','qw','type','map'])

i=0
map_num=0
path=Path

def mapcallback(map_num_row):
    global map_num
    map_num=map_num_row.data

def pathcallback(path_row):
    global path
    path=path_row
#Float32MultiArray lon,lat,alt;
def navcallback(nav_row):
    lon.data.apped(nav_row.longtitude);
    alt.data.apped(nav_row.altitude)


def listener():
    global path
    rospy.init_node('waypoint_manager', anonymous=True)

#    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, callback)
    rospy.Subscriber("/map_num", Int32, mapcallback)
    rospy.Subscriber("waypoint/path",Path,pathcallback)
    #rospy.Subscriber("ublox_gps/fix",NavSatFix,navcallback)
    rospy.spin()
    i=0
    for j in path.poses:
        df.loc[i] = [j.pose.position.x,j.pose.position.y,j.pose.position.z,j.pose.orientation.x,j.pose.orientation.y,j.pose.orientation.z,j.pose.orientation.w,1,1]
        df.to_csv('~/catkin_ws/src/kcctcore/config/waypointdata/wpdata.csv', header=True)
        i+=1
    print(" WayPoint SAVED!!")

if __name__ == '__main__':
    listener()
