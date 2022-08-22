#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import time
import geometry_msgs.msg
from sklearn.neighbors import NearestNeighbors
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from scipy.linalg import pinv
import numpy as np
import math 


class MoveGroupPythonInteface(object):

  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    planning_frame = move_group.get_planning_frame()


    rospy.sleep(2)

    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.53
    p.pose.position.y = -0.4
    p.pose.position.z = 0.257
    scene.add_sphere("prepreka", p, 0.045)

    tic = rospy.get_time()

    r = 0.2
    xs = 0.5
    ys = -0.1
    z = 0.35
    r = 0.2
    xs = 0.53
    ys = -0.2
    z = 0.4 
    #fi = linspace(0,2*pi,200);
    fi = np.linspace(0.0, 2*math.pi, num=80)
    toc = np.empty([1, 3])
    count = 0
    for i in fi:
        
        temp_toc = np.array([r*math.sin(i)+xs, r*math.cos(i)+ys, z - (1.0*(count)/80)*0.3])
        temp_toc = temp_toc.reshape((1,3))
        toc = np.concatenate((toc, temp_toc))
        count +=1

    toc = np.delete(toc, (0), axis=0)

    f = 0
    n = 50

    ##################################################################################################################
    while(1):

        


        tocke = np.empty([1, 3])
        xt = move_group.get_current_pose().pose.position.x
        yt = move_group.get_current_pose().pose.position.y
        zt = move_group.get_current_pose().pose.position.z
        tocke = np.array([xt, yt, zt])
        tocke = tocke.reshape((1,3))
        poc = tocke

        tocke = np.concatenate((tocke, toc))

        col = math.sqrt((xt-p.pose.position.x)**2+(yt-p.pose.position.y)**2+(zt-p.pose.position.z)**2)
        if col < 0.2:
            if xt > p.pose.position.x:
                f = 1.0
            else:  
                f = 0.0
        else:
            f = 0.0

        print col



        #najbliza tocka
        knn = NearestNeighbors(n_neighbors=2)
        knn.fit(tocke)
        a = knn.kneighbors(poc, return_distance=False)

        closest = tocke[a[0][1]]

        #closest[1] = closest[1] -0.4 * f 
        closest[2] = closest[2] + 0.55 * f 


        
        if a[0][1]==80:
            next_closest = tocke[0]
        else:
            next_closest = tocke[a[0][1]+1]

        next_closest[2] = next_closest[2]+ 0.55 * f 

        ####################### vekor najblize tocke na krivulji i iduce tocke
        vec = np.array([next_closest[0]-closest[0], next_closest[1]-closest[1], next_closest[2]-closest[2]])
        d = math.sqrt(vec[0]**2+vec[1]**2+vec[2]**2)
        vecn = 1.0*vec/(1.0 * d)

        ######################## vektor trenutne pozicije i najblize
        vec = np.array([closest[0]-xt, closest[1]-yt, closest[2]-zt])
        d = math.sqrt(vec[0]**2+vec[1]**2+vec[2]**2)
        vecc = 1.0*vec/(1.0*d);

        ######################## rezultantni vektor
        rez = (1 - (1/math.exp(1.2*d)))*vecc + (1/math.exp(1.2*d))*vecn;
        rez = rez / math.sqrt(rez[0]**2+rez[1]**2+rez[2]**2)
        rez = rez * 0.1

        curr = move_group.get_current_joint_values()
        jac = move_group.get_jacobian_matrix(curr)
        invert = pinv(jac)
        a = np.array([[rez[0]],[rez[1]],[rez[2]],[0],[0],[0]])
        t = np.dot(invert,a)
        joint_trenutno = move_group.get_current_joint_values()

        dt = 0.13
        joint_trenutno[0] = (joint_trenutno[0] + t.item(0)*dt)#%2.8973
        joint_trenutno[1] = (joint_trenutno[1] + t.item(1)*dt)#%1.7628
        joint_trenutno[2] = (joint_trenutno[2] + t.item(2)*dt)#%2.8973
        joint_trenutno[3] = (joint_trenutno[3] + t.item(3)*dt)#%-0.0698
        joint_trenutno[4] = (joint_trenutno[4] + t.item(4)*dt)#%2.8973
        joint_trenutno[5] = (joint_trenutno[5] + t.item(5)*dt)#%3.7525
        joint_trenutno[6] = (joint_trenutno[6] + t.item(6)*dt)#%2.8973

        move_group.go(joint_trenutno, wait=False)

        rospy.sleep(0.2)

def main():
  try:
  	MoveGroupPythonInteface()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()