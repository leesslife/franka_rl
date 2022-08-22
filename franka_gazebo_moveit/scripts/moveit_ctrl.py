#!/usr/bin/env python

import sys
import copy
import rospy
import time
import math
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Bool
from moveit_commander.conversions import pose_to_list
from franka_control.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
import pickle
import os

from transforms import quaternion_matrix, quaternion_from_matrix, euler_to_quaternion, quaternion_to_euler

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class plantRecord(object):

  def __init__(self):
    super(plantRecord, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('plant_record',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    self.error_recovery_pub = rospy.Publisher('/franka_control/error_recovery/goal', 
                                                   ErrorRecoveryActionGoal, 
                                                  queue_size=1)

    self.start_recording_pub = rospy.Publisher("/record", Bool, queue_size=1)

    self.touch_id_pub = rospy.Publisher("/touch_id", String, queue_size=1)
    self.touch_id = 0

    self.pose_list = []
    self.pose_start = None

    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.robot = robot
    self.scene = scene
    self.group = group
    self.planning_frame = planning_frame

    self.outfolder = "/home/franka/plantRecord/poses/"

    if not os.path.exists(self.outfolder):
        os.makedirs(self.outfolder)

  def execute_plan(self, plan):
    group = self.group
    return group.execute(plan, wait=True)

  def plan_to_pose(self, p, q):

    group = self.group

    wpose = group.get_current_pose().pose
    wpose.position.x = p.x
    wpose.position.y = p.y
    wpose.position.z = p.z

    wpose.orientation.x = q.x
    wpose.orientation.y = q.y
    wpose.orientation.z = q.z
    wpose.orientation.w = q.w

    (plan, fraction) = group.compute_cartesian_path(
                                       [wpose],   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    time.sleep(0.1)
    if fraction > 0.:
        return self.execute_plan(plan)
    else:
        
        return False


  def loops(self, plantId):

    err_rec_msg = ErrorRecoveryActionGoal()
    msg_true = Bool()
    msg_true.data = True

    no_error = True

    pose = self.pose_start
    success_u = self.plan_to_pose(pose.position, pose.orientation)
    while (success_u == False):
        self.error_recovery_pub.publish(err_rec_msg)
        time.sleep(0.2)
        success_u = self.plan_to_pose(pose.position, pose.orientation)

    for pose in self.pose_list:
        self.touch_id += 1
        success_u = self.plan_to_pose(pose.position, pose.orientation)
        print("Touch id " + str(self.touch_id) + " success: " + str(success_u))
        while (success_u == False):
            self.error_recovery_pub.publish(err_rec_msg)
            time.sleep(0.2)
            success_u = self.plan_to_pose(pose.position, pose.orientation)
            time.sleep(0.4)

        time.sleep(0.5)  
        msg_touchid = String()
        msg_touchid.data = str(1000*plantId + self.touch_id)
        self.touch_id_pub.publish(msg_touchid)
        self.start_recording_pub.publish(msg_true)                
        time.sleep(0.3)  

        filename = self.outfolder + "pose_" + msg_touchid.data + ".txt"

        with open(filename, 'w') as f:
            f.write("position:\n")
            f.write("x: " + str(pose.position.x) + "\n")
            f.write("y: " + str(pose.position.y) + "\n")
            f.write("z: " + str(pose.position.z) + "\n")
            f.write("orientation:\n")
            f.write("x: " + str(pose.orientation.x) + "\n")
            f.write("y: " + str(pose.orientation.y) + "\n")
            f.write("z: " + str(pose.orientation.z) + "\n")
            f.write("w: " + str(pose.orientation.w) + "\n")

            
def main():

  try:
    
    experiment = plantRecord()

    experiment.group.set_max_velocity_scaling_factor(0.1)

    roll = 180
    pitch = 0
    yaw = -45
    q = euler_to_quaternion(yaw, pitch, roll)

    pose_start = experiment.group.get_current_pose().pose

    pose_start.orientation.x = q.x
    pose_start.orientation.y = q.y
    pose_start.orientation.z = q.z
    pose_start.orientation.w = q.w
    
    pose_start.position.x = 0.1
    pose_start.position.y = 0.0
    pose_start.position.z = 0.8

    success = False
    i = 0
    while (success == False) and (i < 5):
        plan, fraction = experiment.group.compute_cartesian_path([pose_start],   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)
        print("fraction:")
        print(fraction)
        execute = raw_input("execute?")
        if (execute == 'y') or (execute == 'Y'):
            success = experiment.execute_plan(plan)
        elif (execute == 'n'):
            return
        i = i + 1

    if i == 5:
        return



    experiment.pose_start = experiment.group.get_current_pose().pose


    time.sleep(2.)

    # record points

    pose_list = []

    while True:
        execute = raw_input("Record new point? [y/n]. No. of points: " + str(len(pose_list)) + "\n")
        if (execute == 'y') or (execute == 'Y'):
            print("\n")
            execute = raw_input("Press ENTER when ready to record pose... ")
            pose_list.append(experiment.group.get_current_pose().pose)
        elif (execute == 'n') or (execute == 'N') or (execute == 'q'):
            print("Done recording. Recorded " + str(len(pose_list)) + " poses.")
            break

    print("Open safety switch.")

    while True:
        execute = raw_input("Ready? [y/n]")
        if (execute == 'y') or (execute == 'Y'):
            err_rec_msg = ErrorRecoveryActionGoal()
            experiment.error_recovery_pub.publish(err_rec_msg)
            break
        else:
            print("retrying... when safety switch opened, type y")            


    
    experiment.pose_list = pose_list

    i_plant = 0
    while True:

        execute = raw_input("Record another plant?   ")
        if (execute == 'y') or (execute == 'Y'):
            print("\n")
            execute = raw_input("Press ENTER when new plant is placed...")
            i_plant += 1
            experiment.loops(i_plant)

        elif (execute == 'n') or (execute == 'N') or (execute == 'q'):
            print("Done recording. Recorded " + str(i_plant) + " plants.")
            break



  except rospy.ROSInterruptException:
      return
  except KeyboardInterrupt:
      print "this is a KeyboardInterrupt that we never catch"
  print("============ Python experiment demo complete!")

  return



if __name__ == '__main__':
    main()
