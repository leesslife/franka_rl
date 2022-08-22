#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import String, Bool

import numpy as np
import open3d as o3d
import ctypes
import struct
import time
from cv_bridge import CvBridge, CvBridgeError
import cv2
import matplotlib.pyplot as plt
import os
# import pcl


class plantRecord():

    def __init__(self):

        rospy.init_node('savePointcloud', anonymous=True)

        # rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback)

        rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.callback_organised)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image,callback=self.convert_depth_image, queue_size=1)
        rospy.Subscriber("/camera/color/image_raw", Image,callback=self.convert_color_image, queue_size=1)



        rospy.Subscriber("/record", Bool, callback=self.record_cb, queue_size=1)
        rospy.Subscriber("/touch_id", String, callback=self.touch_cb, queue_size=1)
        
        self.record = False
        
        self.depth_img = None
        self.touch_id = None
        self.xyz = None

        self.folderpath = "/home/franka/plantRecord/"


    def callback_organised(self, ros_point_cloud):

        gen = pc2.read_points(ros_point_cloud, skip_nans=False)
        int_data = list(gen)

        xyz = np.asarray(int_data)

        self.xyz = xyz[:,0:3]
        
        # rgb = np.zeros(np.shape(xyz))
        
        # out_pcd = o3d.geometry.PointCloud()    
        # out_pcd.points = o3d.utility.Vector3dVector(xyz)
        # out_pcd.colors = o3d.utility.Vector3dVector(rgb)

        # o3d.io.write_point_cloud("/home/marsela/cloud.ply",out_pcd)


    def convert_depth_image(self, ros_image):
        bridge = CvBridge()
        try:
            depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")

        except CvBridgeError, e:
            print e

        depth_array = np.array(depth_image, dtype=np.float32)

        self.depth_img = depth_array


    def convert_color_image(self, ros_image):
        bridge = CvBridge()
        try:
            color_image = bridge.imgmsg_to_cv2(ros_image, "rgb8")
        except CvBridgeError, e:
            print e

        # color_array = np.array(color_image, dtype=np.float32)

        # self.color_img = color_array
        self.color_img = color_image


    def record_cb(self, flag_msg):
        print "Received record flag"
        self.record = True


    def touch_cb(self, touch_msg):
        self.touch_id = touch_msg.data
        print "Received touch id " + self.touch_id



if __name__ == '__main__':
    
    bagRecord = plantRecord()

    rate = rospy.Rate(50)

    depthpath = bagRecord.folderpath + "depth/depth_"
    colorpath = bagRecord.folderpath + "color/color_"
    pcpath = bagRecord.folderpath + "pc/pc_"

    p = bagRecord.folderpath

    p1 = p + "depth/"
    if not os.path.exists(p1):
        os.makedirs(p1)
    p1 = p + "color/"
    if not os.path.exists(p1):
        os.makedirs(p1)
    p1 = p + "pc/"
    if not os.path.exists(p1):
        os.makedirs(p1)


    while not rospy.is_shutdown():
        if (bagRecord.record == True) and (bagRecord.touch_id is not None):

            if (bagRecord.depth_img is not None):
                filepath = depthpath + str(bagRecord.touch_id) + '.jpeg'
                plt.imsave(filepath, bagRecord.depth_img)
            if (bagRecord.color_img is not None):
                filepath = colorpath + str(bagRecord.touch_id) + '.jpeg'
                # plt.imsave(filepath, bagRecord.color_img)
                cv2.imwrite(filepath, bagRecord.color_img)

            if (bagRecord.xyz is not None):
                filepath = pcpath + str(bagRecord.touch_id) + '.ply'

                out_pcd = o3d.geometry.PointCloud()    
                out_pcd.points = o3d.utility.Vector3dVector(bagRecord.xyz)
                rgb = np.zeros(np.shape(bagRecord.xyz))
                out_pcd.colors = o3d.utility.Vector3dVector(rgb)

                o3d.io.write_point_cloud(filepath,out_pcd)

            print "Saved example under id " + bagRecord.touch_id

            bagRecord.record = False
            bagRecord.touch_id = None
            bagRecord.depth_img = None
            bagRecord.color_img = None
            bagRecord.xyz = None

        else:
            rate.sleep()






# def callback(self, ros_point_cloud):

#     print "in callback unorg"
#     xyz = np.depth_imgay([[0,0,0]])
#     rgb = np.depth_imgay([[0,0,0]])
#     gen = pc2.read_points(ros_point_cloud, skip_nans=True)
#     int_data = list(gen)

#     print np.shape(int_data)

#     for x in int_data:

#         if x[2] < 1.5: 
#             xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)

#             test = x[3] 
#             s = struct.pack('>f' ,test)
#             i = struct.unpack('>l',s)[0]
#             pack = ctypes.c_uint32(i).value
#             r = (pack & 0x00FF0000)>> 16
#             g = (pack & 0x0000FF00)>> 8
#             b = (pack & 0x000000FF)

#             rgb = np.append(rgb,[[r,g,b]], axis = 0)
    
#     out_pcd = o3d.geometry.PointCloud()    
#     out_pcd.points = o3d.utility.Vector3dVector(xyz)
#     out_pcd.colors = o3d.utility.Vector3dVector(rgb)

#     o3d.io.write_point_cloud("/home/marsela/cloud.ply",out_pcd)