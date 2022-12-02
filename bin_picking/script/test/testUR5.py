#!/usr/bin/env python

# Ros
import rospy
import rosnode
import tf
from tf.transformations import *

# msg & convert
import geometry_msgs.msg

# Image
import cv2

# 3D
import open3d as o3d

# Utility
import getch
import numpy as np
import time
import copy
import sys
import random



D2R = np.pi/180

class RobotPoseCalibration:
    def __init__(self):
        
        rospy.loginfo(":: Starting RobotPose ::")

        self.pub = rospy.Publisher('ObjTFM', geometry_msgs.msg.Transform, queue_size=10)
        self.Invert_arm_TFM = np.eye(4)
        self.ARoffset_transform = np.eye(4)
        self.Robot_TFM_list = []

    def add_AR_pose(self,Cam_AR_TFM):

        robotpose = np.matmul(Cam_AR_TFM , self.aroffset_transform)
        robotpose = np.matmul(robotpose , self.Invert_arm_TFM)

        self.Robot_TFM_list.append(robotpose)

    def get_robot_pose(self):
        avg_robot_TFM = np.eye(4)
        translation = []
        Rotation = []
        for TFM in self.Robot_TFM_list:
            translation.append(TFM[:3,3])
            Rotation.append(TFM[:3,:3])
        avgTranslation = np.asarray(np.median(translation, axis = 0))
        avg_robot_TFM[:3,3] = avgTranslation.transpose()
        """ AVG ROTATION MATRIX """
        return avg_robot_TFM

    def send_ur5(self,TFM):
        self.Invert_arm_TFM = inverse_matrix(TFM)

        traslation = translation_from_matrix(TFM)
        quaternion = quaternion_from_matrix(TFM)
        TF = geometry_msgs.msg.Transform()
        TF.translation.x = traslation[0]
        TF.translation.y = traslation[1]
        TF.translation.z = traslation[2]
        TF.rotation.x = quaternion[0]
        TF.rotation.y = quaternion[1]
        TF.rotation.z = quaternion[2]
        TF.rotation.w = quaternion[3]
        rospy.loginfo(TF)
        self.pub.publish(TF)
        rospy.sleep(1)

if __name__ == '__main__':

    rospy.init_node("Robottest", anonymous=True)

    ur5 = RobotPoseCalibration()

    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    rgb_image = np.array([])
    while not rospy.is_shutdown():

        key = getch.getch().lower()
        print("key : ",key)
        if key == 't':

            test_matrix = np.array([  [1, 0, 0, random.uniform(0.5, 0.7)],
                                        [0, 1, 0, random.uniform(0.5, 0.7)],
                                        [0, 0, 1, random.uniform(0.5, 0.7)],
                                        [0, 0, 0, 1]],
                                        dtype=float)
            ur5.send_ur5(test_matrix)

        if key == 'q':
            break

cv2.destroyAllWindows()
rospy.spin()