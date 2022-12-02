#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import socket, pickle
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
import time
import getch
import numpy as np
moveit_commander.roscpp_initialize(sys.argv)
import open3d as o3d

import tf
from tf.transformations import *

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()



group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

pose_goal = geometry_msgs.msg.Pose()

isGetPosition=False
isKeep=False

def Rx(theta):
	theta = np.radians(theta)
	return np.matrix([[ 1, 0           , 0           ],
                   [ 0, np.cos(theta),-np.sin(theta)],
                   [ 0, np.sin(theta), np.cos(theta)]])
  
def Ry(theta):
	theta = np.radians(theta)
	return np.matrix([[ np.cos(theta), 0, np.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-np.sin(theta), 0, np.cos(theta)]])
  
def Rz(theta):
	theta = np.radians(theta)
	return np.matrix([[ np.cos(theta), -np.sin(theta), 0 ],
                   [ np.sin(theta), np.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])
    


def talker():
    rospy.init_node('ur5posecheck', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    pose = []
    listener = tf.TransformListener()

    while not rospy.is_shutdown():

        # print(move_group.get_current_pose().pose)

        key = getch.getch().lower()

        print("key : ",key)
        if key == 'j':
            joint_goal = move_group.get_current_joint_values()
            print(f" joint_goal : ",joint_goal)
        if key == 's':

            pose_now = move_group.get_current_pose().pose

            posePrearray = np.array([   pose_now.position.x,pose_now.position.y,pose_now.position.z,
                                        pose_now.orientation.x,pose_now.orientation.y,pose_now.orientation.z,pose_now.orientation.w])
            pose.append(posePrearray)
            print(f" Num pose : ",len(pose))

        if key == 'q':
            for i,x in enumerate(pose):
                print(f"\n:: pose {i+1}: \n",x)
            break
            
        if key == 'p':
            print(f" Preview ")


            pose_now = move_group.get_current_pose().pose
            arm_TFM = concatenate_matrices(translation_matrix((pose_now.position.x,pose_now.position.y,pose_now.position.z)), 
                                                    quaternion_matrix((pose_now.orientation.x,pose_now.orientation.y,pose_now.orientation.z,pose_now.orientation.w)))
            print("arm_TFM : ",np.asarray(arm_TFM))

            Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.5,(0,0,0))

            Robotcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
            Robotcoor.transform(arm_TFM)


            ARCoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
            ARoffset_transform = np.array([[1,0,0,0.2],
                                            [0,1,0,0],
                                            [0,0,1,0],
                                            [0,0,0,1]])
            RotationAR = np.matmul( Ry(-90), Rx(90))
            ARoffset_transform[:3,:3] = RotationAR
            arpose = np.matmul(arm_TFM , ARoffset_transform)
            ARCoor.transform(arpose)

            o3d.visualization.draw_geometries([Realcoor,Robotcoor,ARCoor])

        rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass