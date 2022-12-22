#! /usr/bin/env python

import rospy
import actionlib
import tf
from tf.transformations import *
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

# Moveit
import moveit_commander
import moveit_msgs.msg

# Utility
from utils import *
import numpy as np
import sys

from action_command.msg import *

# Setup Moveit 
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# cam_type = 'sim'
cam_type = 'zivid'
# cam_type = 'azure'

D2R = np.pi/180
R2D = 180/np.pi



class GoplaceServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('go_place', Go_placeAction, self.execute, False)
    
    self.MotionPlan = {
      'prepick' : [0.4189, -1.7976, 2.1467, -1.9373, -1.5358, -0.3840],
      'home' : [0, 0, 0, 0, 0, 0],
      'up' : [0, -1.5707, 0, -1.5707, 0, 0],
      'full_open' : [0],
      'half_open' : [0.4],
      'release_open' : [0.7],
      'close' : [0.8028],
      'grip_close' : [0.789465]
      # 'grip_close' : [0.8028]
      }
    
    self.feedback = Go_placeFeedback()
    self.result = Go_placeResult()
    
    self.server.start()

  def execute(self, goal):
    
    

    print(f" #=#=#=#=#=#=#=#=#=#=#=# goal : {goal.pos_num}  #=#=#=#=#=#=#=#=#=#=#=# ")

    if goal.release_type.data == 'half_open':

      upoffset = np.floor(goal.pos_num/5).astype(int)
      start_prepos = np.array([0.171,-0.151,0.1])
      start_pos = np.array([0.169,-0.153,0.01+(0.008*upoffset)])
      order_offset = np.array([0.0254911995,0.0254911995,0])
    else:
      start_prepos = np.array([0.161,-0.161,0.1])
      start_pos = np.array([0.161,-0.161,0.01])
      order_offset = np.array([0.03,0.03,0])

    pre_placepos = start_prepos+(order_offset*(goal.pos_num%5))
    placepos = start_pos+(order_offset*(goal.pos_num%5))

    rotation = np.eye(4)
    rotation[:3,:3] = np.matmul(Ry(90),Rx(45))

    self.control_plannar_arm(pre_placepos,rotation)
    rospy.sleep(1.)
    self.control_plannar_arm(placepos,rotation)

    self.common_Gripper(goal.release_type.data)

    rospy.sleep(1.)
    self.control_plannar_arm(pre_placepos,rotation)


    # self.feedback.result.data = "6666"
    # self.server.publish_feedback(self.feedback)
    self.result.task_result.data = f"complete : {goal.pos_num}"
    self.server.set_succeeded(self.result)


  def control_plannar_arm(self,position,rotation):

    orientation = tf.transformations.quaternion_from_matrix(rotation)

    gripper_offset = [0,0, 0.165]
    waypoints = []
    pose_goal = move_group.get_current_pose().pose

    #### Single goal point
    
    pose_goal.position.x = position[0]+gripper_offset[0]
    pose_goal.position.y = position[1]+gripper_offset[1]
    pose_goal.position.z = position[2]+gripper_offset[2]

    if pose_goal.position.z < 0.17:
        pose_goal.position.z = 0.17
    pose_goal.orientation.x = orientation[0]
    pose_goal.orientation.y = orientation[1]
    pose_goal.orientation.z = orientation[2]
    pose_goal.orientation.w = orientation[3]
    
    waypoints.append(copy.deepcopy(pose_goal))

    rotation[0,3] = position[0]+gripper_offset[0]
    rotation[1,3] = position[1]+gripper_offset[1]
    rotation[2,3] = position[2]+gripper_offset[2]

    (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.001, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

    # print(f"plan : {len(plan.joint_trajectory.points) }")

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
        # Publish
    display_trajectory_publisher.publish(display_trajectory)

    move_group.execute(plan, wait=True)
    rate.sleep()

  def common_Gripper(self,pose):
    if cam_type == 'sim':
        print(pose)
        joint_goal = group_gripper.get_current_joint_values()
        joint_goal[0] = self.MotionPlan[pose][0]

        group_gripper.go(joint_goal,wait=True)
        group_gripper.stop()
        rate.sleep()
    else:
        command = genCommand(pose)
        pub.publish(command)
        # print("complete publish gripper")
        rospy.sleep(0.2)


if __name__ == '__main__':
  rospy.init_node('go_place_server')
  rate = rospy.Rate(10)
  if cam_type  !='sim':
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    
  control_speed = 0.4
  # ARM
  group_name = "manipulator"
  move_group = moveit_commander.MoveGroupCommander(group_name)

  # save when not use
  move_group.set_max_velocity_scaling_factor(control_speed)
  move_group.set_max_acceleration_scaling_factor(control_speed)

  move_group.set_goal_orientation_tolerance(0.02)
  move_group.set_goal_position_tolerance (0.001)
  move_group.set_planning_time(5)
  move_group.set_num_planning_attempts(10)
  move_group.allow_replanning(True)

  display_trajectory_publisher = rospy.Publisher(
          "/move_group/display_planned_path",
          moveit_msgs.msg.DisplayTrajectory,
          queue_size=20,
      )

  # GRIPPER
  if cam_type =='sim':
    group_name = "gripper"
    group_gripper = moveit_commander.MoveGroupCommander(group_name)
    group_gripper.set_max_velocity_scaling_factor(control_speed)
    group_gripper.set_max_acceleration_scaling_factor(control_speed)
      
  server = GoplaceServer()
  rospy.spin()