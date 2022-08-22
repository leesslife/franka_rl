#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt



class depthImage():

    def __init__(self):

        rospy.init_node('saveDepthimage',anonymous=True)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image,callback=self.convert_depth_image, queue_size=1)

        rospy.spin()

        rospy.Subscriber("/record_frame", Bool, callback=self.record_cb, queue_size=1)

        self.record = False

        # self.stop_recording_pub = rospy.Publisher("/record_stop", Bool, queue_size=1)

        # self.touch_id_pub = rospy.Publisher("/touch_id", String, queue_size=1)
        # self.touch_id = 0

        self.arr = None
        self.touch_id = None

    def convert_depth_image(self, ros_image):
        bridge = CvBridge()
        try:
            depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")

        except CvBridgeError, e:
            print e

        depth_array = np.array(depth_image, dtype=np.float32)

        self.arr = depth_array

    def record_cb(self, flag_msg):
        self.record = True

if __name__ == '__main__':
	depthImg = depthImage()

    rate = rospy.Rate(50)

    prepath = "/home/marsela/folder/image_"
    while not rospy.is_shutdown():
        if (depthImg.record == True) and (depthImg.touch_id is not None) and (depthImg.arr is not None):

            filepath = prepath + str(depthImg.touch_id) + '.jpeg'
            plt.imsave(filepath, depthImg.arr)
            depthImg.record = False
            depthImg.touch_id = None
            depthImg.arr = None

        else:
            rate.sleep()






