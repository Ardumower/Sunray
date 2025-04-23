#!/usr/bin/env python


from __future__ import print_function

#import open3d
from threading import Lock
import rospy
import tf
import numpy as np
import math
import pcl
import matplotlib.pyplot as plt
import struct
import time

# http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
import std_msgs.msg
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Twist, Vector3
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, LaserScan
from geometry_msgs.msg import Point, Quaternion, Pose2D





# https://github.com/strawlab/python-pcl/blob/master/examples/external/ros/ros_utils.py
def cloud_callback(data):    
    global cloud_ref, odom_broadcaster, odom_pub, distlist, fitnesslist, nextPlotTime, fig, axs, cloud_idx, pubCloud, cloudlist
    global pubCloudFiltered, curr_time

    curr_time = rospy.Time.now() #+ rospy.Duration(0.01)
    data_time = data.header.stamp     
    timespan = (curr_time - data_time).to_sec() 
    #print(timespan)
    #if timespan > 0.1: return

    points_list = []
    #for data in point_cloud2.read_points(data, skip_nans=True):
    #    points_list.append([data[0], data[1], data[2], data[3]])
    #pcl_data = pcl.PointCloud_PointXYZRGB()
    #pcl_data.from_list(points_list)
    
    for pt in point_cloud2.read_points(data, skip_nans=True, field_names=("x", "y", "z", "intensity")):
        dist = math.sqrt(pt[0]**2 + pt[1]**2 + pt[2]**2)
        if pt[2] > 0.0: continue        
        if pt[3] < 100: continue
        #if pt[2] < 0.5: continue
        #v = pt[3] * dist        
        #if v < 400: continue        
        points_list.append([pt[0], pt[1], pt[2]])    
    
    #points_list = [[1.0, 1.0, 0.0],[1.0, 2.0, 0.0]]
    #header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'livox_frame'
    #create pcl from points
    pcl = point_cloud2.create_cloud_xyz32(header, points_list)
    #publish    
    pubCloud.publish(pcl)




rospy.init_node('filter', anonymous=True)


pubCloud = rospy.Publisher("/livox/lidar_filtered", PointCloud2, queue_size=100)
subCloud = rospy.Subscriber('/livox/lidar', PointCloud2, cloud_callback, queue_size=100)


if __name__ == '__main__':
    #rospy.spin()
    while not rospy.is_shutdown():
        #mapCloud.header.stamp = rospy.Time.now()
        #pubCloud.publish(mapCloud)
        #map_cloud 
        # https://gist.github.com/lucasw/ea04dcd65bc944daea07612314d114bb   
        rospy.sleep(0.1)

