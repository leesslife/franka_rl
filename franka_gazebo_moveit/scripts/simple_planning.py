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
import pickle

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


class planTrajectory():

  def __init__(self):


    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('plan_trajectory',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

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

  def execute_plan(self, plan):
    group = self.group
    return group.execute(plan, wait=False)

  def plan_to_pose(self, p, q):

    group = self.group

    wpose = geometry_msgs.msg.Pose()
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


def main():

  try:

    experiment = planTrajectory()
    experiment.group.set_max_velocity_scaling_factor(0.5)
    pose_start = experiment.group.get_current_pose().pose


    # kako se planira gibanje u jednu tocku
    # zadaje se pozicija i orijentacija vrha robotske ruke
    new_pose = geometry_msgs.msg.Pose()

    new_pose.orientation.x = -1.
    new_pose.orientation.y = 0.
    new_pose.orientation.z = 0.
    new_pose.orientation.w = 0.
    
    new_pose.position.x = 0.3
    new_pose.position.y = 0.
    new_pose.position.z = 0.7

    success = experiment.plan_to_pose(new_pose.position, new_pose.orientation) # ovaj dio izvrsi plan & execute, planira se gibanje u jednu tocku zadanu sa new_pose

    print("move successful? ", success)

    # robotu treba neko vrijeme da stigne u zadanu tocku, pricekamo da smo sigurni da je tamo 
    time.sleep(2.0)

    print("new pose")
    pose_start = experiment.group.get_current_pose().pose
    print(pose_start)

    print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n")

    # kako se planira kroz vise tocaka - tocke se appendaju u listu tocaka waypoints
    waypoints = []
    new_pose.position.x = pose_start.position.x - 0.05
    waypoints.append(copy.deepcopy(new_pose))

    new_pose.position.y = new_pose.position.y - 0.1
    waypoints.append(copy.deepcopy(new_pose))

    new_pose.position.z += 0.05
    waypoints.append(copy.deepcopy(new_pose))

    # ovo je kao plan dio, predajemo cijelu listu waypoints u kojoj su sve medutocke
    plan, fraction = experiment.group.compute_cartesian_path(waypoints,   
                                       0.01,
                                       0.0)
    print("Which percentage of the trajectory was successfully planned?")
    print(fraction*100,"%")

    # ovo je execute dio 
    experiment.execute_plan(plan)

    time.sleep(0.2)


  except rospy.ROSInterruptException:
      return
  except KeyboardInterrupt:
      print "this is a KeyboardInterrupt that we never catch"
  print("============ Python experiment demo complete!")
  return

if __name__ == '__main__':
    main()
