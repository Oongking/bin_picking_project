#!/usr/bin/env python

# Ros
import rospy
import rosnode
import tf
from tf.transformations import *

# Image
import cv2

# 3D
import open3d as o3d

# Utility
import getch
import numpy as np
import time
import copy

from utils import *

azure = Azure_class()
br = tf.TransformBroadcaster()
listener = tf.TransformListener()
D2R = np.pi/180

while True:
    
    # rgb_image, depth_image = azure.get_rgbd()
    # cv2.imshow('rgb_image', rgb_image)

    br.sendTransform((1.0,1.0,1.0),tf.transformations.quaternion_from_euler(-135*D2R, 0, 135*D2R),rospy.Time.now(),"camera","map")
    br.sendTransform((0.23,0.5,1.0),(-0.612,0.612,-0.354,0.354),rospy.Time.now(),"end_eff","map")
    br.sendTransform((0.0,0.0,0.2),tf.transformations.quaternion_from_euler(90*D2R, 0, -90*D2R),rospy.Time.now(),"AR","end_eff")
    aroffset_transform = concatenate_matrices(translation_matrix((0.0,0.0,0.2)), quaternion_matrix(tf.transformations.quaternion_from_euler(90*D2R, 0, -90*D2R)))
    aroffset_transform = inverse_matrix(aroffset_transform)
    try:
        (ar_trans,ar_rot) = listener.lookupTransform('/camera', '/AR', rospy.Time(0))
        print("ar_trans : ",ar_trans)
        print("ar_rot : ",ar_rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    # print("Matrix : ",translation_matrix(ar_trans), quaternion_matrix(ar_rot))
    ar_transform = concatenate_matrices(translation_matrix(ar_trans), quaternion_matrix(ar_rot))
    
    try:
        (arm_trans,arm_rot) = listener.lookupTransform('/map', '/end_eff', rospy.Time(0))
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    arm_transform = concatenate_matrices(translation_matrix(arm_trans), quaternion_matrix(arm_rot))
    inversed_arm_transform = inverse_matrix(arm_transform)
    robotpose = np.matmul(ar_transform , aroffset_transform)
    robotpose = np.matmul(robotpose , inversed_arm_transform)
    br.sendTransform(translation_from_matrix(robotpose),quaternion_from_matrix(robotpose),rospy.Time.now(),"robot_pose","camera")
    
    # key = cv2.waitKey(1)
    # if key & 0xFF==ord('q'):
    #     break



cv2.destroyAllWindows()