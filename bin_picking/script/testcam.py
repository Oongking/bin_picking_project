#!/usr/bin/env python

# Ros
import rospy
import rosnode
import tf
from tf.transformations import *

# Moveit
import moveit_commander
import moveit_msgs.msg

# msg & convert
import geometry_msgs.msg

# Image
import cv2

# 3D
import open3d as o3d

# Utility
from utils import *
import getch
import numpy as np
import time
import copy
import sys


# camera = 'azure'
camera = 'zivid'

rospy.init_node("test_cam_utils", anonymous=True)

if camera == 'azure':
    cam = Azure_cam()
    matrix_coefficients = azure_matrix_coefficients
    distortion_coefficients = azure_distortion_coefficients

if camera == 'zivid':
    cam = zivid_cam()
    matrix_coefficients = zivid_matrix_coefficients
    distortion_coefficients = zivid_distortion_coefficients

while not rospy.is_shutdown():

    print("=================================================================================")
    print("\n:: Key command ::\n\tc : Capturing & display RGBD\n\tp : Compare build and get\n\te : Shutdown the Server")
    key = getch.getch().lower()
    print("key : ",key)

    if key == 'c':
        rgb_image, depth_image = cam.get_rgbd()
        while not rospy.is_shutdown():
            cv2.imshow("rgb_image",rgb_image)
            cv2.imshow("depth_image",depth_image)

            waitkeyboard = cv2.waitKey(1)

            if waitkeyboard & 0xFF==ord('q'):
                print("===== End =====")
                print(depth_image)
                cv2.destroyAllWindows()
                break

    if key == 't':
        pcd,rgb_image, depth_image,make_pcd = cam.testMatrix()
        config = "eraser_shampoo_3"
        cv2.imwrite(f"/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/{config}/{config}_rgb_img.png",rgb_image)
        cv2.imwrite(f"/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/{config}/{config}_dep_img.png",depth_image)
        o3d.io.write_point_cloud(f"/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/{config}/{config}_pcd_ros.pcd", pcd)
        o3d.io.write_point_cloud(f"/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/{config}/{config}_make_pcd.pcd", make_pcd)

    if key == 'p':
        make_pcd,rgb_image, depth_image = cam.buildPCD()
        Realcoor = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1,(0,0,0))
        o3d.visualization.draw_geometries([make_pcd,Realcoor])
        make_pcd.paint_uniform_color([1, 0.706, 0])
        pcd = cam.get_pcd()
        o3d.visualization.draw_geometries([make_pcd,pcd,Realcoor])
        print("make_pcd : ",make_pcd)
        print("pcd : ",pcd)

    
    if key == 'e':
        
        break

cv2.destroyAllWindows()