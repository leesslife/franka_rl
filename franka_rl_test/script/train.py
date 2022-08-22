from rsa import PublicKey
import rospy
import qlearn
import numpy as np
import rospkg
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import random
import matplotlib.pyplot as plt
import time
import math

pub = rospy.Publisher('/franka/panda_joint7_controller/command', Float64, queue_size=100)
check = rospy.Publisher('/make_check', Bool, queue_size=100)


def deg2rad(angle):
    return angle*math.pi/180

def rad2deg(angle):
    return angle*180/math.pi

def reward_func(angle):
    results=[0.0,1.57,3.14,4.71,6,28]
    dist=[]
    for i in results:
        a=abs(1-angle)
        dist.append(a)
    reward=-1*min(dist)
    return reward

def check_angle(angle):
    if(abs(angle)<0.03):
        return True
    if(abs(angle-1.57)<0.03):
        return True
    if(abs(angle-3.14)<0.03):
        return True
    if(abs(angle-4.71)<0.03):
        return True
    if(abs(angle-6.28)<0.03):
        return True
    return False

def move(state,action):
    global pub  
    global check 
    delta=deg2rad(5)
    pos=None
    while pos is None:
        try:
            pos=rospy.wait_for_message
