#!/usr/bin/env python

from sensor_msgs.msg import PointCloud, PointCloud2, PointField
import std_msgs.msg
from geometry_msgs.msg import Point32
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import time
import rospy


if __name__ == '__main__':
    

    pcd = o3d.io.read_point_cloud("/home/marsela/cloud.ply")

    rospy.init_node('point_polygon_scaler')

    pointcloud_publisher = rospy.Publisher("/my_pointcloud_topic", PointCloud, queue_size=1)

    rospy.sleep(0.5)

    pcl_msg = PointCloud()

    # header = std_msgs.msg.Header()
    # header.stamp = rospy.Time.now()
    # header.frame_id = 'map'
    # pcl_msg.header = header

    for point in pcd.points:
        a = Point32()
        a.x = point[0]
        a.y = point[1]
        a.z = point[2]
        pcl_msg.points.append(a)


    pointcloud_publisher.publish(pcl_msg)

