# Image
import cv2

# 3D
import open3d as o3d

import numpy as np
from utils import *

camera = 'zivid'

if camera == 'zivid':
    boardA3 = zivid_boardA3
    matrix_coefficients = zivid_matrix_coefficients
    distortion_coefficients = zivid_distortion_coefficients
    offset_x = A3_zivid_offset_x
    offset_y = A3_zivid_offset_y


rgb_image = cv2.imread(f'/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/aruco_board/aruco_board_rgb_img.png')

# rvec=None
# tvec=None

# (corners, ids,rejected)= cv2.aruco.detectMarkers(rgb_image,arucoDictA3,parameters= arucoParams)

# if len(corners) > 0:
#     cv2.aruco.drawDetectedMarkers(rgb_image,corners,ids)
#     _,rvec,tvec = cv2.aruco.estimatePoseBoard( corners, ids, boardA3, matrix_coefficients, distortion_coefficients,rvec,tvec)

# transformation_matrix = np.array([  [1, 0, 0, 0],
#                                     [0, 1, 0, 0],
#                                     [0, 0, 1, 0],
#                                     [0, 0, 0, 1]],
#                                     dtype=float)

# transformation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)

# q= tf.transformations.quaternion_from_matrix(transformation_matrix)
# vec= [offset_x,offset_y,0,0]
# global_offset=quaternion_multiply(quaternion_multiply(q, vec),quaternion_conjugate(q))
# tvec[0]=tvec[0]+global_offset[0]
# tvec[1]=tvec[1]+global_offset[1]
# tvec[2]=tvec[2]+global_offset[2]
# transformation_matrix[ :3, 3] = np.asarray(tvec).transpose()

# if rvec is not None and tvec is not None:
#     cv2.aruco.drawAxis( rgb_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.08 )

# while True:
#     cv2.imshow("Original Image",rgb_image)
#     if cv2.waitKey(1) & 0xFF==ord('q'):
#         break

# np.savetxt('/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/workspace_pose.txt', transformation_matrix, delimiter=',',header='real_robotpose Pose')
# load = np.loadtxt('/home/oongking/RobotArm_ws/src/bin_picking/script/test/overlap_dataset/workspace_pose.txt',delimiter=',')
# print("load : ",load)

def workspace_ar_set(rgb_image, camera = 'zivid'):
    if camera == 'zivid':
        boardA3 = zivid_boardA3
        matrix_coefficients = zivid_matrix_coefficients
        distortion_coefficients = zivid_distortion_coefficients
        offset_x = A3_zivid_offset_x
        offset_y = A3_zivid_offset_y
    if camera == 'azure':
        boardA3 = azure_boardA3
        matrix_coefficients = azure_matrix_coefficients
        distortion_coefficients = azure_distortion_coefficients
        offset_x = A3_azure_offset_x
        offset_y = A3_azure_offset_y

    rvec=None
    tvec=None
    (corners, ids,rejected)= cv2.aruco.detectMarkers(rgb_image,arucoDictA3,parameters= arucoParams)

    if len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(rgb_image,corners,ids)
        _,rvec,tvec = cv2.aruco.estimatePoseBoard( corners, ids, boardA3, matrix_coefficients, distortion_coefficients,rvec,tvec)

    transformation_matrix = np.eye(4,dtype=float)
    transformation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
    q= tf.transformations.quaternion_from_matrix(transformation_matrix)

    vec= [offset_x,offset_y,0,0]
    global_offset=quaternion_multiply(quaternion_multiply(q, vec),quaternion_conjugate(q))

    tvec[0]=tvec[0]+global_offset[0]
    tvec[1]=tvec[1]+global_offset[1]
    tvec[2]=tvec[2]+global_offset[2]
    transformation_matrix[ :3, 3] = np.asarray(tvec).transpose()

    if rvec is not None and tvec is not None:
        cv2.aruco.drawAxis( rgb_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.08 )

    while True:
        cv2.imshow("Original Image",rgb_image)
        if cv2.waitKey(1) & 0xFF==ord('q'):
            break
    
    return transformation_matrix


transformation_matrix = workspace_ar_set(rgb_image, camera = 'zivid')