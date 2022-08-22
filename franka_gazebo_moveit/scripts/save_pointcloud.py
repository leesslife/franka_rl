#!/usr/bin/env python

from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import time
import rospy

# import pclpy 
# from pclpy import pcl

import pcl


class plantRecord():

    def __init__(self):

        rospy.init_node('savePointcloud', anonymous=True)

        # rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback)

        rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.callback_organised)



    def callback_organised(self, ros_point_cloud):

        print "in callback organised"
        gen = pc2.read_points(ros_point_cloud, skip_nans=False)
        int_data = list(gen)

        print np.shape(int_data)

        xyz = np.asarray(int_data)

        xyz = xyz[:,0:3]
        rgb = np.zeros(np.shape(xyz))
        
        out_pcd = o3d.geometry.PointCloud()    
        out_pcd.points = o3d.utility.Vector3dVector(xyz)
        out_pcd.colors = o3d.utility.Vector3dVector(rgb)

        o3d.io.write_point_cloud("/home/marsela/cloud.ply",out_pcd)




    def callback(self, ros_point_cloud):

        print "in callback unorg"
        xyz = np.array([[0,0,0]])
        rgb = np.array([[0,0,0]])
        gen = pc2.read_points(ros_point_cloud, skip_nans=True)
        int_data = list(gen)

        print np.shape(int_data)

        for x in int_data:

            if x[2] < 1.5: 
                xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)

                test = x[3] 
                s = struct.pack('>f' ,test)
                i = struct.unpack('>l',s)[0]
                pack = ctypes.c_uint32(i).value
                r = (pack & 0x00FF0000)>> 16
                g = (pack & 0x0000FF00)>> 8
                b = (pack & 0x000000FF)

                rgb = np.append(rgb,[[r,g,b]], axis = 0)
        
        out_pcd = o3d.geometry.PointCloud()    
        out_pcd.points = o3d.utility.Vector3dVector(xyz)
        out_pcd.colors = o3d.utility.Vector3dVector(rgb)

        o3d.io.write_point_cloud("/home/marsela/cloud.ply",out_pcd)

if __name__ == '__main__':
    
    plantRecord()

    time.sleep(10.)

