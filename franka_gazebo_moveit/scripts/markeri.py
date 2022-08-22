#!/usr/bin/env python
 
import roslib;
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray,queue_size=200)

rospy.init_node('register')

markerArray = MarkerArray()

count = 0
MARKERS_MAX = 150
z = 0.33 # 0.33
xs = 0.5
ys = -0.1 #-0.2


r = 0.2
xs = 0.5
ys = 0.2
z = 0.5 



while not rospy.is_shutdown():

	if count==150:
		count = 0

	marker = Marker()
	marker.header.frame_id = "world"
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.scale.x = 0.05
	marker.scale.y = 0.05
	marker.scale.z = 0.05
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 0.0
	marker.color.b = 0.0
	marker.pose.orientation.w = 1.0
 
	marker.pose.position.x = 0.2*(math.sin(  count*2*math.pi/150.0  ))+xs
	marker.pose.position.y = 0.2*(math.cos(  count*2*math.pi/150.0  ))+ys
	marker.pose.position.z = z - (1.0*(count)/150.0)*0.3

	
	if(count > MARKERS_MAX):
	   markerArray.markers.pop(0)

	markerArray.markers.append(marker)

	
	id = 0
	for m in markerArray.markers:
	   m.id = id
	   id += 1

	
	publisher.publish(markerArray)

	count += 1
	rospy.sleep(0.01)

